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

### `eefig_adaptation_analysis.m`

This experiment compares four cases on held-out runs:

- Frozen one-step prediction.
- Predict-then-update one-step prediction.
- Frozen open-loop propagation.
- Open-loop propagation while adapting from arriving measurements.

For adaptive one-step evaluation, the order is always prediction, scoring, and then learning. For adaptive propagation, the propagated state is not reset to the measured state after an update.

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

| Parameter | Current value | Meaning |
|---|---:|---|
| `keep_factor` | `1` | Uses every transition and preserves stream continuity. |
| `filter_window` | `21` | Savitzky-Golay window applied independently to each run. |
| `phi` | `100` | Number of recent transitions used for initialization and offline fitting. At 50 Hz this is 2 s. |
| `min_initial_samples` | `100` | Samples required before creating the first granule. |
| `confidence` | `0.999` | Confidence used for the ellipsoidal admission threshold. |
| `n_anomaly_max` | `5` | Persistent anomaly count used before testing granule creation. |
| `c_separation` | `0.5` | Required separation of a candidate operating region. |
| `rls_mode` | `per_granule` | Gives each local model its own online RLS covariance. |
| `rls_P0` | `10` | Initial online parameter uncertainty for each local model. |
| `rls_forgetting` | `0.99` | Online RLS forgetting factor. Lower values adapt faster but forget sooner. |

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
