# EEFig lateral vehicle model

This folder contains the EEFig model used to learn the lateral vehicle dynamics of the BCN eMotorsport car. It is a work in progress developed as part of a master's thesis.

The model learns the one-step relationship

```text
[vy(k+1); r(k+1)] = F([vy(k); r(k)], [vx(k); delta(k)])
```

where:

- `vy` is lateral velocity in m/s.
- `r` is yaw rate in rad/s.
- `vx` is longitudinal velocity in m/s.
- `delta` is the measured steering angle in rad.

EEFig represents `F` as a collection of local linear Takagi-Sugeno models. Each local model is valid in an ellipsoidal region, called a granule, of the normalized premise space

```text
zeta = [vy; r; vx; delta].
```

For granule `i`, the local consequent is

```text
x_lat(k+1) = A_i*x_lat(k) + B_i*u_lat(k),
x_lat = [vy; r],
u_lat = [vx; delta].
```

The prediction is the membership-weighted blend of every local model:

```text
Abar = sum_i g_i(zeta)*A_i
Bbar = sum_i g_i(zeta)*B_i
x_lat(k+1) = Abar*x_lat(k) + Bbar*u_lat(k).
```

The granules can evolve as new operating regions are observed. Their antecedents describe where each rule applies, while their consequents describe the local dynamics.

## Files in this folder

### `eefig_training.m`

This is the main offline training script and the normal starting point.

It:

1. Declares the training and evaluation dataset files.
2. Calls `prepare_eefig_data` to filter, normalize, and build transitions without joining separate runs.
3. Processes every training run sequentially.
4. Calls `learner.startNewRun()` at each run boundary, clearing short-term temporal memory while preserving the learned model.
5. Evolves the granules and fits their local consequents using moving-window weighted least squares.
6. Evaluates the finished model on both training and held-out evaluation runs without adapting it.
7. Saves the complete trained model as `eefig.mat`.
8. Calls `eefig_model_insights` to print diagnostics and create plots.

Edit the `Configuration` section in this file when changing datasets, filtering, or learning hyperparameters.

### `eefig.mat`

This is the generated model artifact. It contains the variable `eefig_model`, including:

- The trained `EEFIGLearning` object and all its granules.
- The sampling time used during training.
- Training-only normalization scales.
- Training configuration and dataset filenames.
- Training and frozen evaluation metrics.
- Granule and sample counts.

It is overwritten when `eefig_training.m` completes successfully. Runtime functions load this file automatically, so it must exist before using `eefig`, `eefig_matrix`, or online adaptation.

### `eefig.m`

This is the frozen six-state vehicle-model interface used by `compare_models.m`:

```matlab
X_next = eefig(X, U, dt);
```

The expected vectors are:

```text
X = [y; vy; psi; r; delta; delta_dot]
U = [vx; steering_command]
```

It asks `eefig_matrix` for the current affine model and evaluates

```text
X_next = A*X + B*steering_command + C.
```

Calling this function only predicts. It never modifies the learned model.

### `eefig_matrix.m`

This function converts the learned normalized two-state EEFig model into the six-state affine model required by the comparison and MPC code:

```matlab
[A, B, C, g] = eefig_matrix(X, steering_command, vx, dt);
```

It:

- Selects and blends the EEFig local models using `[vy; r; vx; delta]`.
- Converts their matrices from normalized to physical coordinates.
- Rescales the learned discrete increment if `dt` differs from the training sampling time.
- Places the learned `vy` and `r` dynamics in the six-state matrices.
- Adds planar `y` and `psi` kinematics.
- Adds the common second-order steering-actuator model.
- Returns `g`, the membership of every granule at the current operating point.

`vx` is scheduling information and becomes part of the affine term `C`. The learned dynamics use the actual steering state `delta`; the MPC input is the steering command that drives the actuator states.

### `eefig_runtime.m`

This is the persistent model owner. It loads `eefig.mat` once and keeps that model instance in memory.

Supported internal actions are:

- `get`: return the currently loaded model.
- `update`: normalize a measured transition and update the in-memory learner online.
- `reset`: discard the in-memory copy, including all online changes.
- `model_file`: return the path of `eefig.mat`.

Normally, use the public wrapper functions instead of calling this function directly.

### `eefig_online_update.m`

This is the explicit online-adaptation interface:

```matlab
status = eefig_online_update(X_k, U_k, X_k1_measured);
```

It extracts the lateral variables from the six-state vectors, normalizes them with the offline training scales, and calls `EEFIGLearning.updateOnline`. Depending on the data, this can update a local consequent, update granule geometry, or create a new granule.

The update must occur only after `X(k+1)` has actually been measured. Keeping prediction and learning in separate functions prevents evaluation leakage.

### `eefig_reset.m`

This clears the persistent runtime model:

```matlab
eefig_reset();
```

The next prediction reloads the original offline model from `eefig.mat`. Use it before every independent experiment or run when the adaptation history should not carry over.

It does not delete or modify `eefig.mat`.

When an artifact is loaded, the runtime also calls `startNewRun()` once. New
artifacts are saved after the same reset. This prevents an evaluation stream
from inheriting the final training run's finite regression window or anomaly
streak while preserving every learned granule and consequent.

### `compare_models.m` integration

The repository-level comparison script performs the thesis comparison across
all held-out runs. Its frozen EEFig result never updates the model. Its adaptive
result uses strict predict-score-update ordering, reloads the offline artifact
for every run, and freezes the currently adapted model during each 60-step
future rollout. Consequently, adaptation can use only measurements available
before a rollout origin; it cannot learn from that rollout's future targets.

### `eefig_adaptation_analysis.m`

This experiment compares four cases on held-out runs:

- Frozen one-step prediction.
- Predict-then-update one-step prediction.
- Frozen open-loop propagation.
- Open-loop propagation while adapting from arriving measurements.

For adaptive one-step evaluation, the order is always prediction, scoring, and then learning. For adaptive propagation, the propagated `vy` and `r` states are not reset to their measurements after an update. Measured steering angle is imposed at every step as an exogenous vehicle-dynamics input; steering-command and actuator prediction are deliberately excluded from this thesis comparison.

The script resets the saved offline model before each independent experiment. Its configuration also selects the evaluation run, start sample, and propagation horizon.

## Supporting files outside this folder

The EEFig folder provides the model-facing API, but the algorithm implementation and shared preparation utilities live under `mtlb/utils`:

- `prepare_eefig_data.m` loads both splits, checks for path overlap, filters each run independently, constructs valid `k -> k+1` pairs, and obtains normalization scales from training data only.
- `EEFIGLearning.m` manages the complete set of granules, membership calculation, anomaly detection, structural evolution, PJG checks, offline WLS, and online RLS.
- `TSGranule.m` represents one rule: its ellipsoidal antecedent, membership bounds, local matrices, and RLS state.
- `eefig_model_insights.m` reports model structure, held-out errors, extrapolation, and rule usage.
- `models/details.md` contains the full mathematical description shared with the other vehicle models.

## Training the model

### 1. Prepare the common datasets

First generate these files using the repository's dataset-preparation scripts:

```text
mtlb/data/datasets_training.mat
mtlb/data/datasets_evaluation.mat
```

The split must be made by complete runs, not by randomly selecting individual samples. Evaluation paths must not also occur in training. EEFig processes the training runs one after another but resets its finite temporal windows between runs.

### 2. Check the training configuration

Open `eefig_training.m` and review:

```matlab
training_dataset_file = fullfile('data', 'datasets_training.mat');
evaluation_dataset_file = fullfile('data', 'datasets_evaluation.mat');
model_file = fullfile('models', 'eefig', 'eefig.mat');
```

The script currently uses the explicit `mtlb_dir` expected by the project, so it can be launched from another MATLAB working directory.

Important parameters are:

| Parameter | Current value | Meaning | Intuition and training effect |
|---|---:|---|---|
| `keep_factor` | `1` | Uses every available one-step transition. | Increasing it processes fewer pairs, reducing training time but also discarding excitation and making structural updates sparser. It does not change the one-step target interval: every retained sample still targets its immediate successor. |
| `filter_window` | `21` | Savitzky-Golay window applied independently to each run. | A larger window suppresses more measurement noise and usually produces smoother granules, but can blur short transients and reduce the apparent nonlinear dynamics. A smaller window preserves fast behavior but lets more noise drive anomaly detection and parameter fitting. |
| `phi` | `100` | Number of recent transitions used for initialization, offline fitting, and the auxiliary tracker. At 50 Hz this is 2 s. | A larger window gives smoother, more statistically stable local fits but reacts more slowly to a new regime and mixes a wider range of conditions. A smaller window is more local and responsive, but its covariance and consequent estimates are noisier. |
| `min_initial_samples` | `100` | Samples required before creating the first granule and before fitting after a run reset. | Increasing it makes initialization and post-boundary WLS fits better supported, but delays learning at the start of each run. Very small values can create poorly shaped initial granules or ill-conditioned local models. |
| `confidence` | `0.999` | Confidence used to convert the four-dimensional chi-square distribution into the Mahalanobis admission threshold. | Increasing it enlarges the admission ellipsoids, so more samples update existing granules and fewer are labelled anomalous. Decreasing it makes the regions stricter and can create more specialized granules, but may fragment the model. |
| `n_anomaly_max` | `5` | Number of consecutive anomalies that must be exceeded before testing granule creation. | Increasing it requires longer evidence before creating a rule, reducing noise-driven granules but delaying recognition of a genuine new regime. Decreasing it makes the structure grow faster and more readily. |
| `c_separation` | `0.5` | Minimum normalized geometric separation required between a candidate region and every existing granule. | Increasing it makes new-granule creation harder because the candidate must be farther away, generally producing fewer rules. Decreasing it allows closer granules and a more detailed but potentially redundant partition. |
| `rls_mode` | `per_granule` | Chooses a separate online RLS covariance per local model instead of the paper's shared global covariance. | This does not affect offline WLS training. During online adaptation, `per_granule` lets rules retain independent uncertainty and was more stable on the current vehicle data; `global` couples their adaptation through one covariance matrix. |
| `rls_P0` | `10` | Initial online RLS parameter covariance. | This does not affect offline WLS fitting. A larger value makes the first online updates more aggressive because the model initially declares greater uncertainty; an excessively large value can produce parameter jumps. A smaller value adapts more conservatively. |
| `rls_forgetting` | `0.99` | Online RLS forgetting factor. | This does not affect the offline WLS solution. Values closer to `1` retain older information and adapt smoothly; lower values track changes faster but are more sensitive to noise and can forget the offline dynamics too quickly. |

These values are the current real-data configuration, not universal constants. Change one structural parameter at a time and compare held-out error, granule count, evaluation coverage, and propagation stability.

### 3. Run training

From MATLAB, run:

```matlab
run('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb/models/eefig/eefig_training.m')
```

Successful training prints the transitions per run, frozen metrics, the saved model path, and a table describing each granule.

### 4. Read the training insights

Use the outputs as follows:

- **Training vs held-out errors:** similar values indicate reasonable generalization. A much lower training error suggests overfitting or an unrepresentative split.
- **OutsideFraction:** fraction of held-out points outside every learned ellipsoid. A large value means weak operating-region coverage even though the nearest granule can still make a prediction.
- **Granule count:** too few rules may underfit nonlinear behavior; many scarcely used rules may indicate fragmentation or noisy data.
- **EvaluationActivations:** rules with zero held-out activations were not exercised by that split. Their quality cannot be assessed from those evaluation runs.
- **CovarianceCondition:** a very large value signals an elongated or poorly conditioned ellipsoid and deserves inspection.
- **PJG reversions:** antecedent updates rejected because they reduced the granule-quality criterion.
- **Measured/predicted scatter:** a good model follows the identity line without state-dependent bias.
- **Error histograms and P95:** reveal tails that RMSE alone can hide.
- **Four-dimensional granules:** the `vy-r-vx` axes show the physical projection of every Mahalanobis admission ellipsoid, while surface colour represents the corresponding steering-angle coordinate `delta`. Granule numbers mark their physical centres. Because this is a 4-D-to-3-D projection, overlap in the figure does not necessarily mean that the complete 4-D granules overlap.

Do not select a model using only the aggregate RMSE. Compare errors per run and make sure the evaluation set covers the speeds, steering angles, lateral velocities, yaw rates, and track layouts needed by the controller.

## Frozen evaluation

After training, run:

```matlab
run('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb/compare_models.m')
```

`EEFIG offline` uses the saved model without any evaluation-data updates. This is the fair one-step comparison against the offline ANFIS and physics-based models.

The frozen API can also be called directly:

```matlab
eefig_reset();
X_pred = eefig(X_k, U_k, Ts);
```

## Online adaptation

Online evaluation is prequential: predict with the information available at time `k`, score the prediction when time `k+1` arrives, and only then learn from that transition.

```matlab
eefig_reset();

for k = 1:N-1
    X_pred(:, k+1) = eefig(X_measured(:, k), U(:, k), Ts);

    % X_measured(:, k+1) is available only after making the prediction.
    error(:, k+1) = X_pred(:, k+1) - X_measured(:, k+1);
    status(k) = eefig_online_update( ...
        X_measured(:, k), U(:, k), X_measured(:, k+1));
end
```

The returned status reports, among other fields:

- Whether the sample was anomalous.
- Which granule was active.
- Whether a new granule was created.
- Which antecedent updates were accepted or reverted.
- Memberships before and after antecedent updates.

Run `eefig_adaptation_analysis.m` for the prepared frozen-versus-adaptive experiment.

## Understanding offline and online learning

Offline training and online adaptation use the same granule structure but different consequent estimators:

- During **offline training**, samples arrive sequentially so the antecedents and number of granules can evolve. The active local consequent is refitted using a moving window of up to `phi` samples with WLS.
- During **frozen evaluation**, neither antecedents nor consequents change.
- During **online adaptation**, a measured transition updates the evolving structure and the active consequent with RLS. This is causal only when the prediction is made before the update.

The model can therefore be trained once on all training runs, compared fairly in frozen form, and later tested with online adaptation. You do not need a separate model per run. Run boundaries are still essential because anomaly sequences, auxiliary tracking, and WLS windows must not connect the end of one rosbag to the beginning of another.

## Recommended experiment order

1. Train with `eefig_training.m`.
2. Inspect held-out per-run metrics, granule coverage, rule usage, and scatter plots.
3. Run `compare_models.m` for the frozen one-step comparison.
4. Run `eefig_adaptation_analysis.m` to quantify the benefit of adaptation.
5. Compare frozen and adaptive propagation over several runs and horizons.
6. Only after those checks, use the matrices returned by `eefig_matrix` in MPC experiments.

For repeatable results, call `eefig_reset()` between independent runs and never adapt the model before scoring the transition being evaluated.
