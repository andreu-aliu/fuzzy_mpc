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
must be inspected with `visualize_data.m` before drawing conclusions.

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
The present forward-Euler LTV model clamps its lateral scheduling speed to
`3 m/s`, while retaining measured speed in global-position kinematics. The
diagnostic prints measured speed, scheduled speed, Euler and exact spectral
radii, the maximum Euler radius, and the fraction of evaluation samples that
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

Raw error vectors are retained inside these structures, so additional paired
tests, distributions, or case-study plots can be produced without rerunning
every model.

## Recommended interpretation order

For a defensible model comparison:

1. Resolve heading-quality warnings and confirm signal conventions.
2. Inspect operating-condition bin support; do not interpret unsupported bins.
3. Compare one-step RMSE, MAE, and $P_{95}$ against persistence.
4. Check whether conclusions change between sample and equal-run aggregation.
5. Inspect error growth across all horizons, emphasizing 60 steps.
6. Check `ValidFraction`, `BoundedFraction`, maximum errors, and the divergence
   table before ranking models by aggregate RMSE.
7. Compare events and track layouts to identify domain-specific strengths.
8. Use dynamics-only propagation for the principal vehicle-model ranking.
9. Treat end-to-end actuator propagation as a separate diagnostic.
10. Compare frozen and adaptive EEFig only after confirming causal ordering and
    independent reset between runs.

A low one-step error does not guarantee good 60-step propagation. Conversely,
a model with a slightly worse local fit can be preferable for MPC if its bias
and recursive error growth are smaller and it remains stable throughout the
operating domain.
