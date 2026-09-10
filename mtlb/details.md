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

The following results were generated on 10 September 2026 with the current
trained artifacts and `data/datasets_evaluation.mat`. The evaluation set
contains 12 complete runs, 24,871 valid one-step transitions, and 1,215 paired
rolling-window origins. These numbers are a reproducible snapshot of the
current model and dataset versions, not permanent properties of the model
families; this section must be regenerated after retraining a model, changing
the dataset split, or changing preprocessing.

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
| ANFIS LTV residual | 0.0477 | 0.0321 | 0.0455 | 0.0287 | -16.7% | -4.2% |
| EEFig offline | 0.0488 | 0.0411 | 0.0440 | 0.0330 | -19.4% | -33.4% |
| Nonlinear bicycle | 0.2739 | 0.0346 | 0.2575 | 0.0290 | -569.8% | -12.3% |
| Nonlinear double track | 0.1836 | 0.0293 | 0.1551 | 0.0248 | -348.9% | 5.1% |
| LTV MPC | 0.3312 | 0.0470 | 0.3201 | 0.0373 | -709.9% | -52.6% |
| Persistence | 0.0409 | 0.0308 | 0.0383 | 0.0264 | 0% | 0% |

At a 20 ms horizon, persistence is a demanding baseline because the measured
lateral states normally change only slightly between consecutive samples.
ANFIS delta is the strongest local predictor, but its improvement over
persistence is small for $v_y$ (`1.2%`) and clearer for $r$ (`9.6%`). ANFIS
derivative ties its yaw-rate result but is marginally worse in lateral
velocity. The direct model is effectively level with persistence in $v_y$.

The physics-based models have unexpectedly large one-step $v_y$ errors. This
does not by itself show that their recursive dynamics are poor: a constant
measurement offset, state-definition mismatch, unmodelled sensor behaviour,
or imperfect parameter identification can dominate a single 20 ms update.
It does show that their absolute $v_y$ update should be reviewed before making
a claim about physical fidelity from the one-step test.

### Dynamics-only propagation at the MPC horizon

The primary recursive result uses measured speed and steering and propagates
only the predicted dynamics for 60 steps (`1.2 s`).

| Model | $v_y$ sample RMSE [m/s] | $r$ sample RMSE [rad/s] | $v_y$ equal-run RMSE [m/s] | $r$ equal-run RMSE [rad/s] | $v_y$ skill vs persistence | $r$ skill vs persistence | Bounded fraction |
|---|---:|---:|---:|---:|---:|---:|---:|
| ANFIS direct | 0.503 | 0.120 | **0.314** | 0.0849 | 33.4% | 85.0% | 100% |
| ANFIS delta | 0.565 | 0.124 | 0.392 | 0.0871 | 25.2% | 84.4% | 100% |
| ANFIS derivative | 0.702 | 0.142 | 0.532 | 0.0970 | 7.1% | 82.2% | 100% |
| ANFIS LTV residual | 1.487 | 0.554 | 1.790 | 0.6111 | -96.9% | 30.6% | 99.84% |
| EEFig offline | 0.644 | 0.408 | 0.562 | 0.2552 | 14.7% | 48.9% | 100% |
| Nonlinear bicycle | 0.432 | **0.0938** | 0.434 | **0.0666** | 42.8% | **88.2%** | 100% |
| Nonlinear double track | 0.580 | 0.142 | 0.493 | 0.0925 | 23.2% | 82.2% | 100% |
| LTV MPC | **0.427** | 0.0966 | 0.429 | 0.0705 | **43.5%** | 87.9% | 100% |
| Persistence | 0.755 | 0.798 | 0.517 | 0.4780 | 0% | 0% | 100% |

The LTV model has the lowest sample-weighted $v_y$ error, while the nonlinear
bicycle has the lowest yaw-rate error. Their results are very close: LTV is
about `1.2%` better than the nonlinear bicycle in sample-weighted $v_y$, and
the nonlinear bicycle is about `2.9%` better in sample-weighted $r$. The added
double-track complexity does not improve the current result; it is worse than
both bicycle formulations in both propagated states.

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
| Trackdrive | LTV MPC | 0.468 | Nonlinear bicycle | 0.121 |
| Skidpad | ANFIS direct | 0.237 | ANFIS delta | 0.0555 |
| Acceleration | ANFIS direct | 0.248 | ANFIS direct | 0.0556 |
| Autox | ANFIS direct | 0.340 | Nonlinear bicycle | 0.0607 |

These are window-weighted event results. Acceleration has only 77 rolling
windows and very little lateral excitation, so it is useful as a low-excitation
sanity check rather than as decisive evidence about handling dynamics.

### Error growth, stability, and the one-step/propagation distinction

All models except the ANFIS LTV-residual formulation remain within the
configured numerical bounds in every evaluated window and horizon. The
residual model first violates a bound at 40 steps and is bounded in `99.84%`
of the 60-step windows. Its maximum 60-step errors reach `11.67 m/s` in $v_y$
and `3.35 rad/s` in $r$. Its poor aggregate score is therefore not only a plot
scaling artefact, and the model should not be considered MPC-ready in its
current form.

The current LTV low-speed treatment is numerically stable over the diagnostic
grid: the maximum forward-Euler spectral radius is `0.9933`. The 3 m/s
scheduling-speed clamp is active for `4.22%` of evaluation samples. Thus the
previous low-speed numerical instability is controlled without saturating the
predicted outputs.

The most important cross-experiment result is that one-step accuracy does not
predict recursive performance. The bicycle models are poor in one-step $v_y$
but lead the sample-weighted 1.2 s comparison. Conversely, ANFIS delta is the
best one-step model but is worse than ANFIS direct after recursive propagation.
This supports reporting the complete error-versus-horizon curves and using the
60-step result—not one-step training loss alone—when selecting a model for the
MPC.

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
through updates to existing granules, but the result indicates that the
current antecedent coverage accepts all evaluation samples. The combination
of better one-step fit and substantially worse recursive $v_y$ propagation
suggests that the adapted local consequents accumulate bias or produce an
unfavourable closed recursive map. Adaptive EEFig should therefore remain an
experimental comparison, not the MPC model, until its per-granule updates,
normalization, forgetting settings, coverage, and multi-step stability have
been tuned against training-only sequences.

### Steering-actuator diagnostic

At 60 steps every dynamic model has the same steering-position RMSE of
`0.0234 rad`, because they share the same actuator formulation. The principal
model ordering remains broadly similar to the measured-steering experiment,
although individual lateral errors change. This result does not distinguish
the vehicle models' steering behaviour and should remain separate from the
thesis ranking. The measured-steering dynamics-only experiment is the correct
primary comparison for the stated scope.

## Conclusions from the current analysis

- For a sample-weighted 1.2 s MPC horizon, the LTV model is currently the best
  $v_y$ predictor and the nonlinear bicycle is the best yaw-rate predictor.
  Their performance is close enough that model complexity, linearization cost,
  and future MPC integration should be considered alongside RMSE.
- ANFIS direct is the strongest data-driven recursive model and gives the best
  equal-run $v_y$ result. It is also the best $v_y$ model on skidpad,
  acceleration, and autox, but loses to the bicycle models on the long unseen
  `track_4` run. This makes generalization across layouts a central result, not
  a nuisance to average away.
- The nonlinear double-track model does not presently justify its additional
  complexity: it underperforms the nonlinear bicycle at the MPC horizon. Its
  parameters and load-transfer/tire-force assumptions need further
  identification before claiming that the higher-fidelity structure improves
  prediction.
- ANFIS delta is the best one-step formulation, but ANFIS direct propagates
  better. Model selection for MPC must therefore emphasize recursive horizon
  accuracy and boundedness rather than one-step RMSE alone.
- The ANFIS LTV-residual model is both less accurate and the only formulation
  to cross the numerical-error bounds. It requires retraining and stability
  investigation before it can be included as a viable controller model.
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
