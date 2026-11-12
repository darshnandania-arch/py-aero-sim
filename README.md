# Autonomous Vertical Rocket Landing Simulator

A 2-DOF planar flight-dynamics, guidance, navigation and control (GNC) test bed for a
Falcon-9-class booster performing a propulsive vertical landing — the "hover-slam".

The project is deliberately *not* a video game. It is a numerical engineering
simulator: rigid-body dynamics integrated with RK4, a human/PID/LQR controller
trade study, an Apollo-derived ZEM/ZEV guidance layer, a linear Kalman navigation
filter, a Monte Carlo landing-dispersion study, and an automated reporting pipeline.

```
1.2 km altitude · 70 m/s descent · 40 m downrange offset
        │
        ├── coast (engine off, guidance recomputes ignition every cycle)
        │
        ├── LANDING BURN  (ZEM/ZEV, thrust saturated at T_max/m)
        │
        └── TERMINAL FLARE (constant-deceleration profile, 1.5 m/s target)
                                     ↓
                    touchdown 3.0 m/s · 3–4 m from pad
```

---

## Results (reproducible with `python main.py all`)

Four scenarios (nominal, gusty, high-energy, sensor-noise) × four control
architectures, identical wind seeds and initial conditions:

| Metric | `guidance+lqr` | `guidance+pid` | `pid` | `lqr` |
|---|---|---|---|---|
| Landing success | **100 %** | **100 %** | **100 %** | 25 % |
| Touchdown speed [m/s] | 3.01 | 3.07 | 2.49 | 5.06 |
| Landing error [m] | 4.29 | **2.90** | 3.77 | 5.56 |
| Propellant used [kg] | **3806** | 3802 | 4341 | 4119 |
| Time to touchdown [s] | 18.9 | 18.8 | 22.8 | 21.1 |
| Peak load factor [g] | 2.20 | 2.20 | 2.16 | 2.15 |
| Peak tilt [deg] | 18.1 | 17.8 | 9.3 | 15.3 |

Monte Carlo (60 dispersed flights, nominal geometry):

| Controller | Success (95 % CI) | CEP | Touchdown speed p95 | Propellant |
|---|---|---|---|---|
| `guidance+lqr` | 90.0 % (79.9–95.3) | 3.33 m | 4.18 m/s | 3859 ± 121 kg |
| `pid` | 100.0 % (94.0–100.0) | 3.33 m | 3.28 m/s | 4347 ± 129 kg |

**Headline finding:** the ZEM/ZEV guidance layer cuts propellant consumption by
about 12 % and flight time by 17 % relative to a well-tuned cascaded PID, because it
lights the engine at the last kinematically feasible instant instead of following a
fixed velocity profile. The price is a thinner touchdown-speed margin: 6 of 60
dispersed flights exceed the 4 m/s touchdown limit, and closing that gap is the first
item in [`docs/future_work.md`](docs/future_work.md).

Figures: [`outputs/figures/`](outputs/figures) — flight report, dispersion cone,
state history, controller comparison and Monte Carlo panels.
Data: [`outputs/data/`](outputs/data) — full 50 Hz state histories in CSV.

---

## Quick start

```bash
python -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt

python main.py list                     # scenarios and controller modes
python main.py simulate --scenario nominal --mode guidance+lqr --figures
python main.py compare  --scenario gusty
python main.py montecarlo --scenario nominal --mode guidance+lqr --runs 200
python main.py all                      # everything + figures + report.md
python main.py dashboard --mode manual  # live Pygame mission-control display
pytest -q                               # 76 tests, ~8 s
```

Python ≥ 3.10, NumPy, SciPy, Matplotlib. Pygame is only needed for the live
dashboard; the simulator, the analysis pipeline and the whole test suite run headless.

---

## What is modelled

**Plant** (`rocket/physics/`)

* 7-state rigid body: `[x, y, vx, vy, θ, ω, m_fuel]`, integrated with classical RK4
  (10 ms) under a 50 Hz digital GNC loop with zero-order hold.
* ISA atmosphere (two layers, 0–32 km), inverse-square gravity, Mach-dependent drag.
* Thrust lapse with ambient pressure, `Isp`-based mass flow, 40–100 % throttle band
  with a shutdown band, ±5° gimbal with a physical rate limit.
* Mass properties recomputed every step: dry structure + propellant tanks with
  parallel-axis inertia and a moving centre of mass, so `I` and the thrust moment arm
  both change as propellant burns (≈ 40 % drop in `I` over a landing).
* Dryden-equivalent wind: logarithmic mean profile + Ornstein–Uhlenbeck gusts with an
  exact discretisation, plus aerodynamic damping torque.

**GNC** (`rocket/guidance/`, `rocket/control/`)

* Guidance: phase machine (COAST → LANDING_BURN → TERMINAL), suicide-burn ignition
  from constant-deceleration kinematics, ZEM/ZEV powered-descent law, tilt-cone and
  thrust-magnitude saturation with vertical priority.
* Navigation: sensor suite (GNSS 10 Hz, baro/radar altimeter, IMU, attitude reference)
  and two linear Kalman filters (4-state translation, 2-state attitude).
* Control: thrust-vector allocation, PID and LQR attitude regulators (gain-scheduled
  on control effectiveness), cascaded PID and two-level LQR landing controllers, plus
  manual keyboard control.

**Research features** (`rocket/simulation/`)

* Monte Carlo landing dispersion with Wilson confidence intervals and CEP.
* Sensor noise + Kalman filtering (handled above).
* Engine-failure injection (`thrust_scale`), and drag/thrust/wind dispersions.

---

## Repository layout

```
rocket-landing-sim/
├── main.py                    CLI: simulate | compare | montecarlo | all | dashboard
├── rocket/
│   ├── config.py              frozen dataclasses: vehicle, environment, GNC parameters
│   ├── state.py               NavState + angle wrapping
│   ├── physics/
│   │   ├── atmosphere.py      ISA (troposphere + lower stratosphere)
│   │   ├── aero.py            Mach-dependent C_D, drag force
│   │   ├── mass_properties.py mass, CoM, inertia, gimbal moment arm
│   │   ├── wind.py            mean shear + Ornstein–Uhlenbeck gusts
│   │   └── dynamics.py        state derivative, RK4/Euler, environment
│   ├── guidance/
│   │   ├── suicide_burn.py    ignition altitude, stopping distance, time-to-go
│   │   ├── zem_zev.py         ZEM/ZEV law + covariance propagation
│   │   ├── trajectory.py      forward prediction, ballistic impact point
│   │   ├── navigation.py      sensors, linear Kalman filters, nav layer
│   │   └── autopilot.py       phase-scheduled landing guidance
│   ├── control/
│   │   ├── base.py            ControllerBase + GuidanceReference interfaces
│   │   ├── allocation.py      thrust-acceleration → (throttle, pitch command)
│   │   ├── attitude.py        PID / LQR pitch regulators, gain scheduling
│   │   ├── ignition.py        shared suicide-burn gate for the baselines
│   │   ├── pid.py / lqr.py    the two landing controllers
│   │   └── manual.py          keyboard + scripted open-loop control
│   ├── simulation/
│   │   ├── scenarios.py       nominal / gusty / high_energy / noisy
│   │   ├── simulator.py       closed-loop driver, flight log, metrics
│   │   ├── factory.py         mode name → (controller, guidance)
│   │   └── montecarlo.py      dispersion study, Wilson intervals, CEP
│   ├── analysis/              metrics aggregation, CSV/JSON export, Matplotlib figures
│   └── visualization/         Pygame mission-control dashboard
├── tests/                     76 unit / integration / regression tests
├── docs/                      architecture, derivations, roadmap, testing, write-ups
└── outputs/                   generated data, figures, report.md
```

---

## Documentation

| Document | Contents |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | System architecture diagram, layer contracts, dependency rules, SOLID rationale |
| [`docs/derivations.md`](docs/derivations.md) | Full governing equations: dynamics, mass properties, ZEM/ZEV, LQR/CARE, Kalman filter, Monte Carlo statistics |
| [`docs/roadmap.md`](docs/roadmap.md) | 14-week capstone schedule with deliverables and risk register |
| [`docs/implementation_plan.md`](docs/implementation_plan.md) | Seven incremental milestones, each independently verifiable |
| [`docs/testing_strategy.md`](docs/testing_strategy.md) | Test pyramid, verification/validation matrix, regression gates |
| [`docs/portfolio_writeup.md`](docs/portfolio_writeup.md) | Narrative write-up for a portfolio site or interview |
| [`docs/resume_bullets.md`](docs/resume_bullets.md) | Résumé and CV bullets with quantified results |
| [`docs/future_work.md`](docs/future_work.md) | Ten research extensions with the method and expected outcome for each |

---

## Model assumptions and known limitations

Stated explicitly because an engineering model is only as good as its stated limits:

1. **Planar 2-DOF.** Pitch only. No roll/yaw, no lateral-directional coupling, no
   three-dimensional wind.
2. **Point-mass aerodynamics.** Drag only; no lift, no angle-of-attack dependence, no
   centre-of-pressure offset (so no aerodynamic pitch stiffness).
3. **Constant `Isp`.** No mixture-ratio shift, no chamber-pressure transient, no
   engine start/shutdown lag.
4. **Rigid body.** No slosh, no bending modes, no gimbal actuator dynamics beyond a
   10 °/s rate limit.
5. **Linear Kalman filter.** The accelerometer bias is absorbed by process noise
   rather than estimated; an EKF/UKF with bias states is future work.
6. **Open-loop dispersion cone.** The displayed landing ellipse propagates the
   navigation covariance without closed-loop correction, so it is a lower bound.
7. **No landing-leg contact dynamics.** Touchdown is a geometric event with a
   success criterion, not a contact model.

Each of these is a deliberate scope decision, not an oversight; the extensions are
ranked in [`docs/future_work.md`](docs/future_work.md).

---

## References

* Guo, T.-H., *Guidance and Control of Lunar Landing* (ZEM/ZEV derivation and
  Apollo-era powered-descent guidance).
* Zarchan, P., *Tactical and Strategic Missile Guidance* (proportional and
  zero-effort-miss guidance, covariance propagation).
* Bryson, A. E. & Ho, Y.-C., *Applied Optimal Control* (LQR, CARE).
* Crassidis, J. L. & Junkins, J. L., *Optimal Estimation of Dynamic Systems*
  (Kalman filtering, Joseph form).
* Anderson, J. D., *Introduction to Flight* (atmosphere, drag, thrust lapse).
* Etkin, B., *Dynamics of Atmospheric Flight* (rigid-body equations, frames).
* Military Handbook MIL-HDBK-1797 / Dryden turbulence model.
* SpaceX Falcon 9 landing-burn telemetry and published vehicle parameters (public
  domain approximations used for the default configuration).

## Licence

MIT — see `LICENSE`. The vehicle parameters are public-domain estimates, not
proprietary data.
