#!/usr/bin/env python3
"""Command-line entry point for the Autonomous Vertical Rocket Landing Simulator.

Examples
--------
::

    python main.py list
    python main.py simulate --scenario nominal --mode guidance+lqr
    python main.py compare --scenario nominal
    python main.py montecarlo --scenario nominal --mode guidance+lqr --runs 100
    python main.py all                       # everything + figures + report
    python main.py dashboard --mode manual   # live Pygame display (needs a screen)

Every run writes structured data to ``outputs/data`` and figures to
``outputs/figures``.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from rocket.analysis.export import (
    ensure_dir,
    export_comparison,
    export_flight,
    export_monte_carlo,
    export_summary_json,
    write_markdown_report,
)
from rocket.analysis.metrics import aggregate, format_comparison_table
from rocket.analysis.plots import (
    plot_controller_comparison,
    plot_flight_report,
    plot_landing_uncertainty,
    plot_monte_carlo,
    plot_state_history,
)
from rocket.simulation.factory import MODES, build_stack
from rocket.simulation.montecarlo import DispersionConfig, MonteCarloStudy
from rocket.simulation.scenarios import Scenario, all_scenarios
from rocket.simulation.simulator import Simulator

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "outputs")


def _scenario(name: str) -> Scenario:
    registry = all_scenarios()
    if name not in registry:
        raise SystemExit(f"unknown scenario {name!r}; choose from {sorted(registry)}")
    return registry[name]


def _run(scenario: Scenario, mode: str) -> "object":
    controller, guidance = build_stack(mode, scenario)
    return Simulator(scenario, controller, guidance=guidance, seed=1).run()


def _report(result) -> str:
    m = result.metrics
    return (
        f"{result.scenario:<12} {result.controller:<14} {result.outcome:<12} "
        f"V_td {m.touchdown_speed:5.2f} m/s   miss {m.landing_error:5.2f} m   "
        f"prop {m.fuel_used:6.0f} kg   max g {m.max_g_load:4.2f}   "
        f"{'SUCCESS' if m.success else 'FAILURE'}"
    )


# ---------------------------------------------------------------------------
def cmd_list(_args: argparse.Namespace) -> int:
    print("scenarios:")
    for name, scenario in all_scenarios().items():
        print(f"  {name:<12} {scenario.description}")
    print("\ncontroller modes:")
    for mode in MODES:
        print(f"  {mode}")
    return 0


def cmd_simulate(args: argparse.Namespace) -> int:
    scenario = _scenario(args.scenario)
    data_dir = ensure_dir(os.path.join(OUT, "data"))
    fig_dir = ensure_dir(os.path.join(OUT, "figures"))
    modes = [args.mode] if args.mode else list(MODES[:4])

    results = []
    for mode in modes:
        result = _run(scenario, mode)
        results.append(result)
        print(_report(result))
        export_flight(
            result, os.path.join(data_dir, f"flight_{scenario.name}_{mode.replace('+', '_')}.csv")
        )
        if args.figures:
            tag = f"{scenario.name}_{mode.replace('+', '_')}"
            plot_flight_report(result, os.path.join(fig_dir, f"report_{tag}.png"))
            plot_landing_uncertainty(result, os.path.join(fig_dir, f"uncertainty_{tag}.png"))
            plot_state_history(result, os.path.join(fig_dir, f"states_{tag}.png"))

    if len(results) > 1:
        path = plot_controller_comparison(results, os.path.join(fig_dir, f"comparison_{scenario.name}.png"))
        print(f"\ncomparison figure: {os.path.relpath(path, ROOT)}")
    return 0


def cmd_compare(args: argparse.Namespace) -> int:
    scenario = _scenario(args.scenario)
    data_dir = ensure_dir(os.path.join(OUT, "data"))
    fig_dir = ensure_dir(os.path.join(OUT, "figures"))
    modes = args.modes.split(",") if args.modes else list(MODES[:4])
    results = [_run(scenario, mode.strip()) for mode in modes]
    for result in results:
        print(_report(result))
    summary = aggregate(results)
    print("\n" + format_comparison_table(summary))
    export_comparison(results, os.path.join(data_dir, f"comparison_{scenario.name}.csv"))
    path = plot_controller_comparison(results, os.path.join(fig_dir, f"comparison_{scenario.name}.png"))
    print(f"\nfigure: {os.path.relpath(path, ROOT)}")
    return 0


def cmd_montecarlo(args: argparse.Namespace) -> int:
    scenario = _scenario(args.scenario)
    data_dir = ensure_dir(os.path.join(OUT, "data"))
    fig_dir = ensure_dir(os.path.join(OUT, "figures"))
    study = MonteCarloStudy(
        scenario=scenario,
        mode=args.mode,
        n_runs=args.runs,
        seed=args.seed,
        dispersions=DispersionConfig(engine_failure_probability=args.engine_failure),
    )
    started = time.time()
    result = study.run(progress=lambda done, total: print(f"\r  flying {done}/{total}", end="", flush=True))
    print(f"\r  {args.runs} flights in {time.time() - started:.1f} s")
    s = result.summary
    print(
        f"success {100 * s.success_rate:.1f}% "
        f"(95% CI {100 * s.success_rate_ci_low:.1f}-{100 * s.success_rate_ci_high:.1f})\n"
        f"landing error mean {s.landing_error_mean:.2f} m / median {s.landing_error_median:.2f} m / "
        f"p90 {s.landing_error_p90:.2f} m  (CEP {s.cep:.2f} m)\n"
        f"touchdown speed mean {s.touchdown_speed_mean:.2f} m/s / p95 {s.touchdown_speed_p95:.2f} m/s\n"
        f"propellant {s.fuel_used_mean:.0f} +/- {s.fuel_used_std:.0f} kg   "
        f"ignition {s.ignition_altitude_mean:.0f} m"
    )
    export_monte_carlo(result, os.path.join(data_dir, f"montecarlo_{scenario.name}_{args.mode.replace('+', '_')}.csv"))
    path = plot_monte_carlo(result, os.path.join(fig_dir, f"montecarlo_{scenario.name}_{args.mode.replace('+', '_')}.png"))
    print(f"figure: {os.path.relpath(path, ROOT)}")
    return 0


def cmd_all(args: argparse.Namespace) -> int:
    data_dir = ensure_dir(os.path.join(OUT, "data"))
    fig_dir = ensure_dir(os.path.join(OUT, "figures"))

    scenarios = [_scenario(name) for name in ("nominal", "gusty", "high_energy", "noisy")]
    modes = ["guidance+lqr", "guidance+pid", "pid", "lqr"]
    results = []
    for scenario in scenarios:
        for mode in modes:
            result = _run(scenario, mode)
            results.append(result)
            export_flight(
                result,
                os.path.join(data_dir, f"flight_{scenario.name}_{mode.replace('+', '_')}.csv"),
            )
            print(_report(result))

    summary = aggregate(results)
    print("\n" + format_comparison_table(summary))

    figures: list[str] = []
    best = next(r for r in results if r.scenario == "nominal" and r.controller == "guidance+lqr")
    figures.append(plot_flight_report(best, os.path.join(fig_dir, "report_nominal_guidance_lqr.png")))
    figures.append(plot_landing_uncertainty(best, os.path.join(fig_dir, "uncertainty_nominal_guidance_lqr.png")))
    figures.append(plot_state_history(best, os.path.join(fig_dir, "states_nominal_guidance_lqr.png")))
    nominal_results = [r for r in results if r.scenario == "nominal"]
    figures.append(plot_controller_comparison(nominal_results, os.path.join(fig_dir, "comparison_nominal.png")))
    export_comparison(results, os.path.join(data_dir, "flights_comparison.csv"))
    export_summary_json(results, os.path.join(data_dir, "summary.json"))

    mc_results = []
    for scenario_name, mode, runs in (
        ("nominal", "guidance+lqr", args.runs),
        ("nominal", "pid", args.runs),
    ):
        study = MonteCarloStudy(_scenario(scenario_name), mode=mode, n_runs=runs, seed=args.seed)
        mc = study.run(progress=lambda done, total: print(f"\r  {scenario_name}/{mode}: {done}/{total}", end="", flush=True))
        print("")
        export_monte_carlo(
            mc, os.path.join(data_dir, f"montecarlo_{scenario_name}_{mode.replace('+', '_')}.csv")
        )
        figures.append(
            plot_monte_carlo(mc, os.path.join(fig_dir, f"montecarlo_{scenario_name}_{mode.replace('+', '_')}.png"))
        )
        mc_results.append(mc)
        print(
            f"  {scenario_name}/{mode}: success {100 * mc.summary.success_rate:.0f}% "
            f"CER {mc.summary.cep:.2f} m"
        )

    report = write_markdown_report(
        results, os.path.join(OUT, "report.md"), monte_carlo=mc_results, figures=figures
    )
    print(f"\nreport: {os.path.relpath(report, ROOT)}")
    print(f"figures: {len(figures)} written to {os.path.relpath(fig_dir, ROOT)}")
    return 0


def cmd_dashboard(args: argparse.Namespace) -> int:
    from rocket.visualization.dashboard import run_dashboard

    scenario = _scenario(args.scenario)
    result = run_dashboard(scenario=scenario, mode=args.mode, real_time=not args.fast)
    if result is not None:
        print(_report(result))
    return 0


# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rocket-lander",
        description="Autonomous Vertical Rocket Landing Simulator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", help="list scenarios and controller modes")
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("simulate", help="fly one scenario")
    p.add_argument("--scenario", default="nominal")
    p.add_argument("--mode", default="guidance+lqr")
    p.add_argument("--figures", action="store_true", help="also render report figures")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("compare", help="fly every controller on one scenario")
    p.add_argument("--scenario", default="nominal")
    p.add_argument("--modes", default="", help="comma separated; default all four")
    p.set_defaults(func=cmd_compare)

    p = sub.add_parser("montecarlo", help="run a dispersed landing study")
    p.add_argument("--scenario", default="nominal")
    p.add_argument("--mode", default="guidance+lqr")
    p.add_argument("--runs", type=int, default=100)
    p.add_argument("--seed", type=int, default=2026)
    p.add_argument("--engine-failure", type=float, default=0.0)
    p.set_defaults(func=cmd_montecarlo)

    p = sub.add_parser("all", help="full study: scenarios x controllers + Monte Carlo + report")
    p.add_argument("--runs", type=int, default=60)
    p.add_argument("--seed", type=int, default=2026)
    p.set_defaults(func=cmd_all)

    p = sub.add_parser("dashboard", help="live Pygame mission-control display")
    p.add_argument("--scenario", default="nominal")
    p.add_argument("--mode", default="guidance+lqr")
    p.add_argument("--fast", action="store_true", help="do not throttle to real time")
    p.set_defaults(func=cmd_dashboard)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
