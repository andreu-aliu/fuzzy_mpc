# Vehicle-dynamics model comparison

This document describes the evaluation procedure implemented by
[`compare_models.m`](compare_models.m). It is the comparison-methodology part
of the project documentation:

- [`data/details.md`](data/details.md) explains where the datasets come from,
  how complete runs are split, and how the ROS signals are processed.
- [`models/details.md`](models/details.md) defines every model and its
  equations.
- This file explains how those models are compared and how to interpret the
  resulting tables and figures.

The primary goal is to compare lateral vehicle-dynamics estimation. Steering
actuator prediction is deliberately kept out of the primary ranking.

## Running the comparison

First generate the training and evaluation datasets and train the models. Then
run:

```matlab
run('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb/compare_models.m')
```

The script uses the explicit project path, changes MATLAB's working directory
to `mtlb`, and adds the MATLAB project tree to the path. It reads
`data/datasets_evaluation.mat`; training data is never loaded by the comparison
script.

The main configuration is at the top of `compare_models.m`:

| Setting | Current value | Purpose |
|---|---:|---|
| `Ts` | `0.02 s` | Dataset and model sample time. |
| `cfg.horizons` | `[1 5 10 20 40 60]` | Reported recursive prediction horizons. |
| `cfg.focus_horizon` | `60` | Main MPC horizon: 60 samples or 1.2 s. |
| `cfg.rolling_stride` | `20` | Starts a new rollout every 0.4 s. |
| `cfg.min_bin_samples` | `50` | Minimum support required to display an operating-condition bin. |
| `cfg.run_end_to_end` | `true` | Enables the separate steering-actuator experiment. |
| `cfg.run_adaptive_eefig` | `true` | Enables causal adaptive EEFig evaluation. |
| `cfg.plot_selected_window` | `true` | Enables the individual propagation case-study figure. |
| `cfg.selected_run` | `1` | Evaluation-run index used by the case study. |
| `cfg.selected_start` | `1001` | First selected-run sample in the case study. |
| `cfg.selected_horizon` | `300` | Number of samples propagated in the current case study (6 s). This does not change the 60-step MPC comparison horizon. |
| `cfg.divergence_abs_vy` | `10 m/s` | Diagnostic numerical-divergence threshold. |
| `cfg.divergence_abs_r` | `5 rad/s` | Diagnostic numerical-divergence threshold. |

All evaluation runs must contain finite `time`, `psi`, `vx`, `vy`, `r`,
`delta`, and `st` signals. Their stored sample time must match `Ts`.

## Models under comparison

The current comparison contains:

1. ANFIS direct next-state model.
2. ANFIS state-increment model.
3. ANFIS derivative model.
4. ANFIS correction of the LTV residual.
5. Frozen offline EEFig.
6. Nonlinear bicycle model.
7. Nonlinear double-track model.
8. Linear time-varying bicycle model used by the MPC.
9. Persistence/constant-yaw-rate baseline.

Every model receives the same measured initial condition and exogenous inputs
at a given comparison sample. Model colours are fixed and reused in every
figure.

The ANFIS LTV-residual artifact must be retrained whenever the underlying LTV
equations or low-speed clamp changes. Until then, its learned correction and
runtime base model describe different residuals and its results are not valid
for the final comparison.

## Evaluation data and preprocessing

The comparison uses complete held-out runs. It does not create a random sample
split, and no evaluation run is used to identify offline model parameters.
This avoids leakage between highly correlated neighbouring samples.

The script consumes the uniformly resampled signals stored in the evaluation
MAT file. It does not reapply the centred Savitzky--Golay filters used by model
training scripts. This choice has two consequences:

- Results measure performance against the signal stream available to the
  controller rather than against a non-causally smoothed target.
- Individual training scripts' filtered validation losses are not directly
  comparable with the raw-stream metrics printed by `compare_models.m`.

If common filtering is later added to the comparison, it must be causal for an
online claim and must be applied identically to every model and baseline.

## Heading reference and validation

`psi` is the recorded planar body heading extracted from
`/as/c/state.odom.heading`. It is wrapped to a planar angle while reading the
rosbag, unwrapped before interpolation, and made relative to the beginning of
each selected run.

SLAM `x/y` is not used to reconstruct heading. Consequently, SLAM position
drift cannot contaminate the heading comparison, and trajectory course is not
mistaken for vehicle body yaw when sideslip is present.

Before comparing models, the script checks

$$
\frac{\psi_{k+1}-\psi_k}{T_s}
\quad\text{against}\quad
\frac{r_k+r_{k+1}}{2}.
$$

For each run it reports correlation, RMSE, bias, and 95th-percentile absolute
rate error. A warning is issued when the correlation is below `0.8`, is not
finite, or rate RMSE exceeds `0.15 rad/s`. Heading results from a warned run
must be inspected with `visualize_raw_data.m` before drawing conclusions.

All heading prediction errors are wrapped using

$$
e_\psi=\operatorname{atan2}(\sin(\hat\psi-\psi),
\cos(\hat\psi-\psi)),
$$

so crossing $-\pi/\pi$ cannot create an artificial $2\pi$ error.

## Experiment 1: one-step prediction

One-step evaluation tests local model accuracy without recursive error
accumulation. Every valid transition in every held-out run is used.

At sample $k$, the model state and input are constructed as

$$
x_k=[0,\ v_y(k),\ \psi(k),\ r(k),\ \delta(k),\ 0]^\mathsf T,
$$

$$
u_k=[v_x(k),\ \delta(k)].
$$

The measured steering position is placed in the steering state. Setting the
steering-rate state to zero and passing the same steering position as the
command places the shared actuator subsystem at equilibrium. Therefore it
cannot influence the predicted one-step lateral dynamics.

Predicted $v_y(k+1)$ and $r(k+1)$ are compared with their measurements. The
one-step heading update is also evaluated internally, although it is a
kinematic integration of yaw rate rather than an independently learned output.

The one-step figures contain:

- Overall RMSE, MAE, and 95th-percentile absolute error.
- RMSE versus measured longitudinal speed.
- RMSE versus absolute measured steering angle.
- The sample count supporting every operating-condition bin.
- Separate RMSE values for each held-out run.

Event and track-layout tables make it possible to see whether an apparently
good overall score is dominated by one type of manoeuvre or one circuit.

## Experiment 2: dynamics-only rolling propagation

This is the primary recursive comparison for the thesis. It asks how errors
accumulate when each model propagates its own predicted lateral state while
receiving the measured exogenous variables that define the vehicle operating
condition.

For every rollout origin $k_0$:

1. Initialise predicted $v_y$ and $r$ from their measurements at $k_0$.
2. Set predicted relative heading to zero.
3. At every future step, overwrite the steering state with measured
   $\delta(k)$ and set steering rate to zero.
4. Pass measured $v_x(k)$ and measured $\delta(k)$ to the model.
5. Propagate the model's predicted $v_y$, $r$, and relative heading without
   resetting them to measurements.
6. Score the prediction at each configured horizon.

Thus, this experiment is recursive in the states being studied but conditional
on the measured speed and steering trajectory:

$$
[\hat v_y(k+1),\hat r(k+1)]
=f(\hat v_y(k),\hat r(k),v_x(k),\delta(k)).
$$

It does not test steering-command tracking. A poor steering actuator cannot be
mistaken for poor vehicle dynamics, and a good actuator cannot improve the
dynamics score.

### Rolling origins and paired horizons

A rollout starts every `cfg.rolling_stride` samples in every held-out run. Only
origins having at least 60 future samples are used. The same origins therefore
contribute to 1-, 5-, 10-, 20-, 40-, and 60-step metrics.

This pairing is important: a change in RMSE with horizon represents recursive
error growth over the same collection of situations, not a changing sample
population near run boundaries.

The main reported horizon is 60 samples:

$$
60T_s=60(0.02)=1.2\ \mathrm{s},
$$

which matches the current MPC prediction horizon.

### Selected propagation-window case study

Aggregate errors show overall performance but do not reveal the shape of an
individual rollout. The final comparison section therefore reconstructs one
editable dynamics-only window using `cfg.selected_run`, `cfg.selected_start`,
and `cfg.selected_horizon`.

The figure overlays measured and predicted $v_y$, $r$, and relative heading
for every frozen model, persistence, and adaptive EEFig. It also shows the
measured $v_x$ and $\delta$ sequence supplied to all models. Sample zero of
every predicted state trajectory equals the measurement at the selected
origin; subsequent samples are recursive model predictions.

For the adaptive EEFig curve, a fresh offline artifact is loaded and updated
chronologically using only transitions before `cfg.selected_start`. If the
origin is sample $k_0$, the adaptation set is exactly

$$
\left\{v_y(k),r(k),v_x(k),\delta(k),v_y(k+1),r(k+1)
\right\}_{k=1}^{k_0-1}.
$$

The adapted parameters are then frozen for the displayed future window. The
figure reports the number of earlier transitions used and the number of new
granules created before the window. No measured future $v_y$ or $r$ is used to
adapt the plotted prediction.

The selected window must satisfy

$$
\texttt{selected\_start}+\texttt{selected\_horizon}
\leq N_{\mathrm{run}}.
$$

Changing these three configuration values is the intended way to inspect a
failure, transient, event, or operating region found in the aggregate tables.

## Persistence and constant-yaw-rate baseline

The baseline holds the lateral states constant:

$$
\hat v_y(k+h)=v_y(k),\qquad
\hat r(k+h)=r(k),
$$

and integrates heading using the initial constant yaw rate:

$$
\hat\psi(k+h)-\psi(k)=hT_s r(k).
$$

The baseline establishes whether a model provides useful dynamics rather than
merely benefiting from the short horizon and slowly changing measurements.
Skill is defined as

$$
\operatorname{skill}=100
\frac{\operatorname{RMSE}_{\mathrm{persistence}}
-\operatorname{RMSE}_{\mathrm{model}}}
{\operatorname{RMSE}_{\mathrm{persistence}}}.
$$

Positive skill means the model improves on persistence. Negative skill means
that simply retaining the current state predicts better.

## Experiment 3: steering-command/actuator propagation

This is a separate diagnostic and is not part of the primary vehicle-dynamics
ranking.

At every origin it starts from measured `delta` and a steering rate estimated
with `gradient(delta,Ts)`. It then passes

$$
u_k=[v_x(k),\mathrm{steering\ command}(k)]
$$

and allows each model's steering states to propagate without overwriting them
with measured steering. The script reports lateral-state, heading, and
steering-position errors.

Comparing this result with dynamics-only propagation separates two effects:

- A large degradation only in the end-to-end test indicates steering-actuator
  mismatch.
- Similar errors in both tests point to the vehicle-dynamics formulation.

Since this thesis focuses on vehicle dynamics and the models currently share a
simple nominal steering actuator, conclusions about model quality should come
from the dynamics-only result.

## Experiment 4: frozen and adaptive EEFig

Frozen EEFig uses the saved offline artifact without modifying it.

Adaptive EEFig is evaluated causally and independently in every held-out run:

1. Reload the same saved offline model.
2. Clear its finite temporal window, anomaly counter, and auxiliary tracker.
3. Predict the next measured transition.
4. Score that prediction.
5. Only after scoring, update EEFig with the newly available measured target.

This is predict-then-update evaluation. The evaluated sample never trains the
prediction that is scored for that same sample.

For a recursive rollout, the EEFig parameters available at its origin are
frozen throughout that hypothetical future prediction. Measurements arriving
later in the real run may update the model for later origins, but future
targets are never used inside a 60-step rollout. This reproduces how an
adaptive model could be used inside an MPC: adapt between controller calls,
then keep the prediction model fixed while solving one horizon.

The script reports new granules per evaluation run. A zero count does not mean
that no adaptation occurred; existing local consequents and antecedents can be
updated without creating a new granule.

## Error metrics

For errors $e_j$, sample RMSE, MAE, and 95th-percentile absolute error are

$$
\operatorname{RMSE}=\sqrt{\frac{1}{N}\sum_j e_j^2},\qquad
\operatorname{MAE}=\frac{1}{N}\sum_j|e_j|,
$$

$$
P_{95}=\operatorname{percentile}_{95}(|e_j|).
$$

RMSE emphasizes occasional large errors, MAE describes typical magnitude, and
$P_{95}$ exposes the upper error tail without being controlled by a single
maximum.

Propagation summaries additionally include maximum absolute error,
`ValidFraction`, and `BoundedFraction`:

- `ValidFraction` is the fraction with finite $v_y$ and $r$ errors.
- `BoundedFraction` is the fraction that is finite and satisfies
  $|e_{v_y}|\leq10$ m/s and $|e_r|\leq5$ rad/s.

The bounded thresholds are numerical-failure diagnostics. Predictions are not
saturated, removed, or replaced before any error metric is calculated.

## Sample, run, and declared-lap aggregation

The script intentionally reports more than one aggregation because the held-out
runs have very different durations.

### Sample-weighted

All transitions or rolling windows are concatenated before calculating RMSE.
Long runs contribute more because they contain more evaluated situations.

### Equal-run

An RMSE is calculated separately for each run and those RMSE values are
averaged:

$$
E_{\mathrm{run}}=\frac{1}{R}\sum_{i=1}^{R}E_i.
$$

Every rosbag has equal influence regardless of its duration.

### Declared-lap weighted

Each run-level RMSE is weighted by the manually declared number of laps:

$$
E_{\mathrm{lap\ weighted}}
=\frac{\sum_i L_iE_i}{\sum_iL_i}.
$$

This is not genuine lap-by-lap evaluation. The MAT files store a run's lap
count but not the sample indices or timestamps at which laps begin and end. A
ten-lap trackdrive therefore contributes ten times the weight of a one-lap
autox run, but all ten laps inherit the same complete-run RMSE. True per-lap
statistics require a recorded lap counter, lap timestamps, or manually
specified lap boundaries.

For thesis conclusions, sample-weighted and equal-run results should be the
primary views. Declared-lap weighting is a useful sensitivity analysis.

## Event and track-layout grouping

The script reports one-step and 60-step dynamics-only RMSE grouped by event and
track layout. These tables remain sample/window weighted within each group.

Track layouts that occur only in training or only in evaluation cannot support
a within-layout generalization claim. Consult the split summary in
[`data/details.md`](data/details.md) before attributing a difference to model
quality rather than track coverage.

## Divergence and plot scaling

A model is listed in the numerical-divergence table when any horizon has a
`BoundedFraction` below one. The table gives its first divergent horizon,
bounded fractions, and maximum errors.

Large failures are never deleted from numerical results. To keep other models
visible, a plot axis is limited to the normal range only when a plotted value
exceeds ten times the median of that plot's finite non-negative values. The
figure states how many values are off scale and directs the reader to the full
tables.

The comparison also sweeps the LTV lateral-state spectral radius over speed.
The LTV model clamps its lateral scheduling speed to `3 m/s`, while retaining
measured speed in global-position kinematics. The diagnostic prints measured
speed, scheduled speed, the rejected forward-Euler radius, the implemented
exact-ZOH radius, their maxima, and the fraction of evaluation samples that
activate the clamp.

Output clamping is not used. Saturating predicted $v_y$ or $r$ would hide model
instability and could make an invalid model appear accurate.

## Figures produced

The script creates:

1. One-step yaw-rate and lateral-velocity error summaries.
2. One-step residuals against speed and steering magnitude, plus bin support.
3. One-step RMSE for each held-out run.
4. Dynamics-only error-versus-horizon curves for $v_y$, $r$, and heading using
   sample and equal-run aggregation.
5. The sample-, run-, and declared-lap-weighted comparison at 60 steps.
6. Equivalent end-to-end propagation curves.
7. A dedicated steering-position error-versus-horizon plot.
8. Frozen-versus-adaptive EEFig one-step and propagation plots.
9. A selected-window overlay of measured and propagated states for all models.

The same colour always identifies the same model. The persistence baseline uses
a dashed line where line styles are available.

## Results retained in the MATLAB workspace

After execution, the following variables remain available:

| Variable | Contents |
|---|---|
| `heading_quality` | Per-run heading/yaw-rate consistency metrics. |
| `one_step` | One-step errors, operating-condition inputs, and summary table. |
| `dynamics` | Dynamics-only rolling errors, origins, summaries, and divergence table. |
| `end_to_end` | Steering-command propagation results and divergence table. |
| `eefig_adaptive` | Causal adaptive EEFig errors, summaries, and new-granule counts. |
| `focus_dynamics` | Dynamics-only model table at the selected 60-step horizon. |
| `focus_end_to_end` | End-to-end table at the selected 60-step horizon. |
| `ltv_stability` | LTV spectral-radius and low-speed-clamp diagnostics. |
| `selected_window` | Measured inputs/states and every model trajectory for the configured case study. |

Raw error vectors are retained inside these structures, so additional paired
tests, distributions, or case-study plots can be produced without rerunning
every model.

## Current results

The following results were regenerated on 14 September 2026 with the model
artifacts currently on disk and `data/datasets_evaluation.mat`. The evaluation
set contains 12 complete runs, 24,871 valid one-step transitions, and 1,215
paired rolling-window origins. Each rolling origin is shared by every model.

These numbers are a snapshot, not permanent properties of the model families.
All ANFIS variants and EEFig were retrained from the current training MAT file
on 14 September 2026. Both nonlinear physical-model trainers were also rerun;
their multi-horizon candidates improved $v_y$ but worsened $r$ at 60 steps, so
the two-state acceptance rule correctly retained the nominal parameter sets.

### Heading-signal validation

The recorded heading does not currently pass the consistency thresholds on
all held-out runs. Depending on the run, the correlation between
$d\psi/dt$ and measured yaw rate ranges from `0.471` to `0.967`, and the rate
RMSE ranges from `0.082` to `0.430 rad/s`. Every evaluation run triggers at
least one of the current correlation or RMSE warning criteria.

Consequently, $v_y$ and $r$ are the valid primary outputs for the present
comparison. Heading RMSE is retained as a diagnostic, but it should not be
used to rank models or support thesis conclusions until the heading signal,
time alignment, frame convention, and yaw-rate sign/latency have been checked.
The particularly low acceleration-run correlations may partly reflect the
small yaw-rate excitation, for which correlation is intrinsically fragile,
but this does not explain the large rate RMSE in several cornering runs.

### One-step prediction

| Model | $v_y$ sample RMSE [m/s] | $r$ sample RMSE [rad/s] | $v_y$ equal-run RMSE [m/s] | $r$ equal-run RMSE [rad/s] | $v_y$ skill vs persistence | $r$ skill vs persistence |
|---|---:|---:|---:|---:|---:|---:|
| ANFIS direct | 0.0410 | 0.0303 | 0.0379 | 0.0231 | -0.3% | 1.7% |
| ANFIS delta | **0.0404** | **0.0279** | **0.0378** | **0.0229** | **1.2%** | **9.6%** |
| ANFIS derivative | 0.0411 | **0.0279** | 0.0380 | **0.0229** | -0.5% | **9.6%** |
| ANFIS LTV residual | 0.0418 | 0.0308 | 0.0380 | 0.0239 | -2.1% | -0.1% |
| EEFig offline | 0.0488 | 0.0411 | 0.0440 | 0.0330 | -19.4% | -33.4% |
| Nonlinear bicycle | 0.2814 | 0.0439 | 0.2658 | 0.0363 | -588.2% | -42.3% |
| Nonlinear double track | 0.1836 | 0.0293 | 0.1551 | 0.0248 | -348.9% | 5.1% |
| LTV MPC | 0.2260 | 0.0486 | 0.2144 | 0.0356 | -452.7% | -57.7% |
| Persistence | 0.0409 | 0.0308 | 0.0383 | 0.0264 | 0% | 0% |

At a 20 ms horizon, persistence is a demanding baseline because the measured
lateral states normally change only slightly between consecutive samples.
ANFIS delta is the strongest local predictor, but its improvement over
persistence is small for $v_y$ (`1.2%`) and clearer for $r$ (`9.6%`). ANFIS
derivative ties its yaw-rate result but is marginally worse in lateral
velocity. The direct model is effectively level with persistence in $v_y$.

The physics-based models have unexpectedly large one-step $v_y$ errors. This
does not by itself show that their recursive dynamics are poor: parameter
mismatch, the measured $v_y$ definition, sensor noise, or a small systematic
one-step bias can dominate this metric. It does show that their absolute
$v_y$ update and state conventions should be reviewed before claiming physical
fidelity from the one-step result alone.

The operating-condition plots show that the four ANFIS formulations remain
close to the persistence baseline across the well-populated speed and steering
bins. The physical-model $v_y$ residuals are much larger and generally grow
with steering magnitude. Results above approximately `21 m/s` and at the
largest steering magnitudes have little support, so isolated changes in those
bins are not strong evidence of generalization or failure.

### Dynamics-only propagation at the MPC horizon

The primary recursive result uses measured speed and steering and propagates
only the predicted dynamics for 60 steps (`1.2 s`).

| Model | $v_y$ sample RMSE [m/s] | $r$ sample RMSE [rad/s] | $v_y$ equal-run RMSE [m/s] | $r$ equal-run RMSE [rad/s] | $v_y$ skill vs persistence | $r$ skill vs persistence | Bounded fraction |
|---|---:|---:|---:|---:|---:|---:|---:|
| ANFIS direct | 0.503 | 0.120 | **0.314** | 0.0849 | 33.4% | 85.0% | 100% |
| ANFIS delta | 0.565 | 0.124 | 0.392 | 0.0871 | 25.2% | 84.4% | 100% |
| ANFIS derivative | 0.702 | 0.142 | 0.532 | 0.0970 | 7.1% | 82.2% | 100% |
| ANFIS LTV residual | 1.014 | 0.312 | 1.158 | 0.3750 | -34.3% | 61.0% | 100% |
| EEFig offline | 0.644 | 0.408 | 0.562 | 0.2552 | 14.7% | 48.9% | 100% |
| Nonlinear bicycle | **0.427** | **0.0928** | 0.431 | **0.0671** | **43.4%** | **88.4%** | 100% |
| Nonlinear double track | 0.580 | 0.142 | 0.493 | 0.0925 | 23.2% | 82.2% | 100% |
| LTV MPC | **0.423** | 0.0962 | 0.428 | 0.0702 | **44.0%** | 87.9% | 100% |
| Persistence | 0.755 | 0.798 | 0.517 | 0.4780 | 0% | 0% | 100% |

Every model now remains bounded on every evaluated window. LTV has the lowest
sample-weighted $v_y$ RMSE (`0.423 m/s`), while the nonlinear bicycle has the
lowest sample-weighted yaw-rate RMSE (`0.0928 rad/s`). ANFIS direct has the
lowest equal-run $v_y$ RMSE and remains the strongest data-driven recursive
model. The nonlinear double-track model is worse than the simpler nonlinear
bicycle in both aggregate propagated states, so its additional structure has
not yet produced an overall accuracy benefit.

The ranking changes under equal-run aggregation. ANFIS direct produces the
lowest equal-run $v_y$ RMSE, whereas the nonlinear bicycle remains best for
yaw rate. This difference is important because the single trackdrive run
provides 553 of 1,215 windows (`45.5%`) and therefore strongly affects the
sample-weighted result. Moreover, that run is on `track_4`, which has no
training laps in the current split. The sample-weighted table consequently
contains a substantial out-of-layout generalization test; it should not be
presented as though all layouts had equal training coverage.

The event results confirm that there is no universal winner:

| Held-out event | Best $v_y$ model at 1.2 s | RMSE [m/s] | Best $r$ model at 1.2 s | RMSE [rad/s] |
|---|---|---:|---|---:|
| Trackdrive | LTV MPC | 0.461 | Nonlinear bicycle | 0.121 |
| Skidpad | ANFIS direct | 0.237 | ANFIS delta | 0.0555 |
| Acceleration | ANFIS direct | 0.248 | ANFIS direct | 0.0556 |
| Autox | ANFIS direct | 0.340 | Nonlinear bicycle | 0.0613 |

These are window-weighted event results. Acceleration has only 77 rolling
windows and very little lateral excitation, so it is useful as a low-excitation
sanity check rather than as decisive evidence about handling dynamics.

### Error growth, stability, and the one-step/propagation distinction

No model violates the configured numerical-error bounds in the regenerated
comparison. In particular, the LTV model is bounded in all 1,215 rolling
windows. The stability diagnostic explains the correction: the rejected
20 ms forward-Euler transition has spectral radius `1.1181` at the `3 m/s`
schedule floor, whereas the implemented exact affine ZOH transition has radius
`0.1533`. Its maximum radius over the diagnostic speed grid is `0.8190`.
Thus the former catastrophic LTV errors were numerical discretization errors,
not evidence that the continuous-time bicycle equations were unstable.

The retrained LTV-residual model is also bounded everywhere, but its 60-step
sample RMSE (`1.014 m/s`, `0.312 rad/s`) is still worse than its exact-ZOH LTV
base. A locally useful residual correction therefore does not automatically
form a stable or accurate recursive correction.

The most important cross-experiment result is that one-step accuracy does not
predict recursive performance. LTV is much worse than persistence at one step
but is best in sample-weighted $v_y$ at 60 steps. Conversely, ANFIS delta is
the best one-step model but is worse than ANFIS direct after recursive
propagation, and the ANFIS derivative formulation accumulates error fastest
among the direct data-driven formulations. This supports reporting the complete
error-versus-horizon curves and using the 60-step result—not one-step training
loss alone—when selecting a model for the MPC.

### Frozen versus causally adaptive EEFig

| Evaluation | EEFig version | $v_y$ sample RMSE | $r$ sample RMSE | Heading sample RMSE |
|---|---|---:|---:|---:|
| One step | Frozen | 0.0488 m/s | 0.0411 rad/s | -- |
| One step | Adaptive | **0.0453 m/s** | **0.0328 rad/s** | -- |
| 60 steps | Frozen | **0.644 m/s** | 0.408 rad/s | 0.379 rad |
| 60 steps | Adaptive | 1.154 m/s | **0.389 rad/s** | **0.283 rad** |

Causal adaptation improves one-step EEFig RMSE by approximately `7.2%` in
$v_y$ and `20.4%` in $r$. It does not yet beat persistence at one step. At the
MPC horizon it improves yaw-rate RMSE by about `4.7%` and the diagnostic
heading RMSE by about `25%`, but worsens $v_y$ RMSE by about `79%`. The adaptive
$v_y$ result is also `52.8%` worse than persistence.

No new granules are created in any held-out run. Adaptation is still occurring
through consequent and antecedent updates to existing granules, but structural
evolution is not exercised by this split under the current thresholds. The
combination of better one-step fit and substantially worse recursive $v_y$
propagation suggests that the adapted local consequents accumulate bias or
produce an
unfavourable closed recursive map. Adaptive EEFig should therefore remain an
experimental comparison, not the MPC model, until its per-granule updates,
normalization, forgetting settings, coverage, and multi-step stability have
been tuned against training-only sequences.

The selected six-second trackdrive window reinforces this warning but is only
a case study, not an aggregate metric. Causally adaptive EEFig departs strongly
in both states; the ANFIS derivative and LTV-residual formulations also develop
large biases. Several other models follow the change in yaw rate but retain a
$v_y$ offset. Since six seconds is five times the MPC horizon, this plot is
useful for diagnosing drift rather than ranking the controller models.

### Steering-actuator diagnostic

At 60 steps every dynamic model has the same steering-position RMSE of
`0.0234 rad`, because they share the same actuator formulation. The vehicle
errors change because every model receives the same simulated rather than
measured steering, but the test cannot distinguish their steering behaviour.
It should remain separate from the thesis ranking. The measured-steering
dynamics-only experiment is the correct primary comparison for the stated
scope.

## Conclusions from the current analysis

- At the 1.2 s MPC horizon, exact-ZOH LTV gives the lowest sample-weighted
  $v_y$ RMSE (`0.423 m/s`) and the nonlinear bicycle gives the lowest yaw-rate
  RMSE (`0.0928 rad/s`). Their results are close enough that both are useful
  physical references; the choice depends on state weighting and computational
  requirements rather than a single universal winner.
- ANFIS direct is the strongest data-driven recursive model and gives the best
  equal-run $v_y$ result. It is also the best $v_y$ model on skidpad,
  acceleration, and autox, but loses to the bicycle models on the long unseen
  `track_4` run. This makes generalization across layouts a central result, not
  a nuisance to average away.
- The nonlinear double-track model does not presently justify its additional
  complexity in aggregate: it underperforms the nonlinear bicycle at the MPC
  horizon. It does show isolated strengths, including one-step yaw rate and
  track-specific cases, so the appropriate conclusion is that its current
  identification is incomplete—not that double-track dynamics are inherently
  inferior.
- ANFIS delta is the best one-step formulation, but ANFIS direct propagates
  better. Model selection for MPC must therefore emphasize recursive horizon
  accuracy and boundedness rather than one-step RMSE alone.
- The ANFIS LTV-residual model is bounded after retraining against the corrected
  base, but it is less accurate recursively than plain LTV. It requires a
  multi-step or stability-aware residual objective before it can be considered
  a viable controller model.
- The earlier catastrophic LTV result was caused by forward-Euler
  discretization at the low-speed schedule clamp. Exact affine ZOH removes the
  instability: all rollouts are bounded and LTV becomes the best aggregate
  $v_y$ predictor at 60 steps. This is a concrete implementation bug, not a
  property of the LTV model family.
- Frozen EEFig is stable but not competitive with the leading models. Causal
  adaptation improves local and yaw-rate prediction, yet seriously degrades
  recursive $v_y$. The current adaptive configuration has not demonstrated
  the multi-step behaviour required for MPC.
- The recorded heading is not sufficiently consistent with yaw rate for a
  defensible heading ranking. Present thesis conclusions should be restricted
  to measured $v_y$ and $r$ until this data-quality issue is resolved.
- These conclusions apply to the current held-out split. Because run duration,
  event counts, and track coverage are unbalanced, both sample-weighted and
  equal-run results must be reported, together with event/layout breakdowns.
- Before freezing the thesis tables, resolve the heading-quality warnings and
  record the dataset, artifact, and comparison configuration versions. The
  current data-driven artifacts have already been retrained from the same
  training MAT file and the LTV discretization has been corrected.

## Recommended interpretation order

For a defensible model comparison:

1. Confirm that every artifact was trained from the recorded training MAT file
   version; this was done for the current 14 September 2026 snapshot.
2. Confirm LTV discrete stability over its full scheduled-speed range; exact
   affine ZOH is now implemented and passes the current diagnostic.
3. Resolve heading-quality warnings and confirm signal conventions.
4. Inspect operating-condition bin support; do not interpret unsupported bins.
5. Compare one-step RMSE, MAE, and $P_{95}$ against persistence.
6. Check whether conclusions change between sample and equal-run aggregation.
7. Inspect error growth across all horizons, emphasizing 60 steps.
8. Check `ValidFraction`, `BoundedFraction`, maximum errors, and the divergence
   table before ranking models by aggregate RMSE.
9. Compare events and track layouts to identify domain-specific strengths.
10. Use dynamics-only propagation for the principal vehicle-model ranking.
11. Treat end-to-end actuator propagation as a separate diagnostic.
12. Compare frozen and adaptive EEFig only after confirming causal ordering and
    independent reset between runs.

A low one-step error does not guarantee good 60-step propagation. Conversely,
a model with a slightly worse local fit can be preferable for MPC if its bias
and recursive error growth are smaller and it remains stable throughout the
operating domain.

## Closed-loop MPC benchmark

`simulate_mpc.m` supports both detailed single-scenario inspection and an
all-scenario held-out benchmark. The relevant configuration is at the top of
the script:

```matlab
run_selection = "all";       % or a numeric vector such as [1 3 5]
use_full_run = true;         % false uses idx_start/window_sample_count
trim_start_seconds = 0;
trim_end_seconds = 0;
max_mpc_windows_per_run = 500; % Inf evaluates the complete run
plot_run_indices = [];       % e.g. [1 5] to plot only those batch runs
```

With `run_selection = "all"`, every run in `datasets_evaluation.mat` is
simulated using the same prediction model, plant, horizon, weights, scales,
and constraints. EEFig is reset at the beginning of every run, so adaptive
evaluation does not transfer held-out information between scenarios.
`run_mpc_scenario.m` contains the complete preparation and closed-loop loop for
one run; `simulate_mpc.m` calls it only from the `SIMULATE` section.

The script sections are deliberately independent. `LOAD DATA` only loads and
validates the selected run indices. `SIMULATE` produces `scenario_results` but
does not create the main result figures. `PLOT INPUT DATA` and `PLOT RESULTS`
read the already-computed results and display only the dataset run indices in
`plot_run_indices`. `PERFORMANCE INSIGHTS` reads those same immutable result
structures, updates the MAT database, and rebuilds the aggregate table. No
loop or conditional block crosses a `%%` section boundary.

`max_mpc_windows_per_run` limits the number of MPC optimizations performed in
each run after trimming. A value of `500`, for example, evaluates 500
consecutive closed-loop control instants (10 s at 20 ms) from every selected
run. `Inf` restores complete-run evaluation. The windows must remain
consecutive: skipping intermediate control instants would break the propagated
closed-loop state and would no longer represent one continuous experiment.

The spatial reference uses the measured longitudinal-speed profile, but
clamps negative reference progress to zero. Several runs begin with small
negative `vx` values while stationary (sensor noise rather than intentional
reverse driving). Allowing those values to integrate arc length would query
the path at `s < 0` and create a non-finite MPC reference. The measured speed
passed to the simulated plant is not replaced by this reference clamp.

The script stores two variables in `mpc_results.mat`:

- `mpc_results_database` contains one row per controller/setup and scenario.
- `mpc_results_summary` groups all available scenarios belonging to the same
  controller/setup and evaluation protocol.

The controller key includes the dataset file, full-run versus selected-window
protocol, per-run window cap, trimming policy, prediction and plant models,
EEFig mode, horizon, sample time, scales, weights, limits, and reference
method. The scenario key contains the run, actual start sample, sample count,
event, layout, and declared lap count. Repeating the same trial updates its
database row; changing the scenario adds a row under the same controller key.

Manual selected windows, capped-run benchmarks, and complete-run benchmarks
deliberately receive different controller keys and are never combined. The summary reports
`ScenarioCoveragePercent`; thesis-level aggregate results should only be used
when this is 100%. Its principal lateral metric is the equal-run mean of the
run RMSE values. It also reports the median and worst run and a sample-weighted
RMSE, together with model error, computation time, deadline misses, and solver
failures.

`DeclaredLaps` remains metadata only. A ten-lap run still produces one
scenario-level measurement because the dataset has no lap boundary indices.
Consequently, the closed-loop summary is genuinely run-balanced, not
lap-balanced.

## Automatic plot export

Every main plotting script automatically exports each completed figure as a
200 dpi PNG under

```text
mtlb/plots/<generating-script>/
```

For example:

```text
mtlb/plots/compare_models/one_step_model_errors.png
mtlb/plots/simulate_mpc/closed_loop_aggregate_model_comparison.png
mtlb/plots/visualize_raw_data/physical_consistency.png
mtlb/plots/visualize_data/evaluation_run_01_trackdrive_track_4.png
mtlb/plots/eefig_training/eefig_training_evolution.png
```

Names come from the MATLAB figure `Name` property and are normalized to
lowercase filesystem-safe identifiers. If several figures in one export have
the same name, deterministic `_02`, `_03`, ... suffixes are added. Running a
script again replaces the corresponding existing PNG so the folder represents
the latest result.

`utils/save_script_figures.m` implements the shared export behavior. It is
called after figures are complete by `compare_models`, each plot section of
`simulate_mpc`, the prepared-data view in `visualize_data`, every diagnostic
plot section of `visualize_raw_data`, the ANFIS and EEFig
training/analysis scripts, both nonlinear physical-model identification
scripts, and `simulator_error`. Iterative MPC debug figures are exported once
at the end of `SIMULATE`, not at every controller iteration.

### Closed-loop model comparison section

After storing the scenario results for every desired prediction model, run
`CLOSED-LOOP MODEL COMPARISON` by itself. It reads `mpc_results.mat`; it does
not rerun the controller or alter the database.

Controller results are comparable only when dataset, evaluation protocol,
window cap, trimming, plant, horizon, sample time, weights, scales, limits,
and reference construction are identical. The section constructs comparison
groups by holding all those fields fixed and allowing only
`PredictionModel` and `EEFigAdaptive` to change. It first prints the available
groups. By default,

```matlab
comparison_group_id = [];
comparison_only_complete = true;
```

selects the most recently updated group containing at least two model variants
and excludes variants whose `ScenarioCoveragePercent` is below 100%. Set an
explicit group number to inspect an older setup. Setting
`comparison_only_complete = false` is useful while accumulating results, but
incomplete variants must not be used for the final ranking.

The section produces:

- a ranked numerical table with coverage, equal-run and sample-weighted
  lateral RMSE, median and worst-run RMSE, tail error, model error,
  computation time, deadline misses, and solver failures;
- an aggregate figure comparing tracking accuracy, tail error, online time,
  and the accuracy/computation trade-off;
- a run-by-model lateral-RMSE heatmap;
- a run-by-model deadline-miss heatmap.

The equal-run lateral RMSE is the principal ranking metric. The per-run
heatmap and worst-run values must be inspected before accepting that ranking,
because an aggregate improvement can hide failure on one event or layout.
