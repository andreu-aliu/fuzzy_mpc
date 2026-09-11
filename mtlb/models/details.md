# Vehicle-model mathematical details

This document is the mathematical reference for every model implemented in
`mtlb/models`. It describes the equations that the MATLAB code currently
executes, not only the intended model architecture.

Related documentation: [`../data/details.md`](../data/details.md) describes
the datasets and [`../details.md`](../details.md) describes the held-out model
comparison procedure.

> **Maintenance rule:** any change to a model equation, state, input, parameter,
> numerical limit, discretization, training target, or linearization must update
> this document in the same change.

## Model inventory

| Model | Type | Lateral-dynamics representation | Current role |
|---|---|---|---|
| `ltv` | Physics-based, linear time-varying | Linear single-track bicycle | Baseline and MPC-ready |
| `nonlinear_bicycle` | Physics-based, nonlinear | Nonlinear single-track with saturated tyre forces | Comparison model |
| `nonlinear_double_track` | Physics-based, nonlinear | Four independent tyres and lateral load transfer | Higher-fidelity comparison model |
| `anfis_delta` | Data-driven, local affine | Learns one-sample changes in lateral velocity and yaw rate | Comparison candidate and MPC-ready |
| `anfis_direct` | Data-driven, local affine | Learns next lateral velocity and yaw rate directly | Comparison candidate and MPC-ready |
| `anfis_dot` | Data-driven, local affine | Learns lateral-velocity and yaw-rate derivatives | Comparison candidate and MPC-ready |
| `anfis_residuals` | Hybrid physics/data-driven | Learns the one-step error of `ltv` | Comparison candidate and MPC-ready |
| `eefig` | Data-driven, evolving TS fuzzy | Online ellipsoidal granules with local linear consequents | Comparison candidate, online-adaptive and MPC-ready |

`anfis_residuals` and `anfis_residuals_2` use the same trained residual model.
The first evaluates the nonlinear ANFIS output directly; the second uses its
local affine matrix form. They are implementation alternatives, not separate
models in the comparison table.

The former `ltv_tv` model is not part of the model set. Torque vectoring is out
of scope and none of the models in this document has an external yaw-moment
input.

## Common interface and notation

All one-step model functions use

$$
x_{k+1}=f(x_k,U_k,T_s),
$$

with state

$$
x=\begin{bmatrix}y&v_y&\psi&r&\delta&\dot\delta\end{bmatrix}^{\mathsf T}
$$

and function input

$$
U=\begin{bmatrix}v_x&u_\delta\end{bmatrix}^{\mathsf T}.
$$

The symbols are:

| Symbol | Meaning | Unit |
|---|---|---|
| $y$ | Lateral position in the local/global propagation frame | m |
| $v_y$ | Vehicle lateral velocity at the centre of gravity | m/s |
| $\psi$ | Vehicle heading in that frame | rad |
| $r=\dot\psi$ | Yaw rate | rad/s |
| $\delta$ | Actual front-wheel steering angle | rad |
| $\dot\delta$ | Steering-angle rate | rad/s |
| $v_x$ | Measured/prescribed longitudinal speed | m/s |
| $u_\delta$ | Requested steering angle | rad |
| $T_s$ | Runtime integration/prediction step | s |

Longitudinal speed is a measured scheduling variable, not a predicted state or
an MPC decision variable. The MPC decision input is only $u_\delta$.

Every model shares the exact planar kinematics

$$
\dot y=v_x\sin\psi+v_y\cos\psi, \qquad \dot\psi=r,
$$

and the second-order steering actuator

$$
\ddot\delta=\omega_n^2(u_\delta-\delta)
-2\zeta\omega_n\dot\delta,
$$

where $\omega_n=16\ \mathrm{rad/s}$ and $\zeta=0.5$. Steering angles are
limited to $\lvert\delta\rvert,\lvert u_\delta\rvert\leq0.45\ \mathrm{rad}$
in the nonlinear models. Unless stated otherwise, continuous equations are
discretized by forward Euler:

$$
x_{k+1}=x_k+T_s\dot x_k.
$$

## Linear time-varying bicycle model (`ltv`)

### Lateral dynamics

The model assumes small tyre slip and replaces the two tyres on each axle by
one axle force. Its effective axle cornering stiffnesses are

$$
C_f=2B_fC_f^{\mathrm{tyre}}D_f^{\mathrm{tyre}},\qquad
C_r=2B_rC_r^{\mathrm{tyre}}D_r^{\mathrm{tyre}}.
$$

In the code these are approximately obtained from
$B_f=B_r=10.5507$, $C_f^{\mathrm{tyre}}=C_r^{\mathrm{tyre}}=1.2705$,
$D_f^{\mathrm{tyre}}=1104\ \mathrm N$, and
$D_r^{\mathrm{tyre}}=1281.5\ \mathrm N$.

The forward-Euler implementation uses the low-speed scheduling clamp

$$
v_{x,d}=\max(v_x,3\ \mathrm{m/s}).
$$

The model does not support reverse-driving dynamics, so negative and small
positive measurements use the same positive floor. The lateral equations are

$$
\dot v_y=-\frac{C_f+C_r}{m v_{x,d}}v_y
+\left(\frac{C_r l_r-C_f l_f}{m v_{x,d}}-v_{x,d}\right)r
+\frac{C_f}{m}\delta,
$$

$$
\dot r=\frac{C_r l_r-C_f l_f}{I_z v_{x,d}}v_y
-\frac{C_f l_f^2+C_r l_r^2}{I_z v_{x,d}}r
+\frac{C_f l_f}{I_z}\delta.
$$

The $-v_{x,d}r$ term is the body-frame Coriolis contribution under this
low-speed approximation. Measured $v_x$ remains in the global $y$ kinematics;
only the lateral dynamic scheduling value is clamped. Nominal parameters
are $m=215\ \mathrm{kg}$, $I_z=188\ \mathrm{kg\,m^2}$, and
$l_f=l_r=0.765\ \mathrm m$.

### Local affine form

`ltv_matrix` returns

$$
x_{k+1}=A_dx_k+B_du_{\delta,k}+C_d.
$$

The lateral-position equation is affine-linearized about the supplied
operating point $(\bar v_y,\bar\psi)$:

$$
\dot y\approx
\cos\bar\psi\,v_y
+(v_x\cos\bar\psi-\bar v_y\sin\bar\psi)\psi+c_y,
$$

where

$$
c_y=v_x\sin\bar\psi+\bar v_y\cos\bar\psi
-\cos\bar\psi\,\bar v_y
-(v_x\cos\bar\psi-\bar v_y\sin\bar\psi)\bar\psi.
$$

This affine term makes the approximation exact at the operating point.
Forward-Euler discretization gives

$$
A_d=I+T_sA_c,\qquad B_d=T_sB_c,\qquad C_d=T_sC_c.
$$

The model is LTV because $v_x$, $\bar v_y$, and $\bar\psi$ can change at each
prediction step.

## Nonlinear bicycle model (`nonlinear_bicycle`)

### Slip angles and tyre forces

The front and rear axle slip angles are

$$
\alpha_f=\delta-\operatorname{atan2}(v_y+l_fr,v_{x,s}),
$$

$$
\alpha_r=-\operatorname{atan2}(v_y-l_rr,v_{x,s}),
$$

with $v_{x,s}=\max(|v_x|,v_{x,\min})$ and
$v_{x,\min}=0.5\ \mathrm{m/s}$.

Each complete axle uses a simplified Pacejka law:

$$
F_{yf}=D_f\sin\!\left(C_f\arctan(B_f\alpha_f)\right),
$$

$$
F_{yr}=D_r\sin\!\left(C_r\arctan(B_r\alpha_r)\right).
$$

Here $D_f$ and $D_r$ represent two-tyre axle peak forces, not per-tyre
forces. The nominal values are $D_f=2208\ \mathrm N$ and
$D_r=2563\ \mathrm N$.

### Vehicle dynamics

The nonlinear lateral and yaw equations are

$$
\dot v_y=\frac{F_{yf}\cos\delta+F_{yr}}{m}-v_xr,
$$

$$
\dot r=\frac{l_fF_{yf}\cos\delta-l_rF_{yr}}{I_z}.
$$

Together with the common kinematic and actuator equations, these form the
six-state derivative integrated by forward Euler. This model captures tyre
force saturation and exact slip-angle geometry, but does not distinguish the
left and right wheels.

### Parameter identification

`nonlinear_bicycle_training.m` fits

$$
\theta=\begin{bmatrix}B_f&B_r&D_f&D_r\end{bmatrix}
$$

by bounded nonlinear least squares against measured one-step
$v_y(k+1)$ and $r(k+1)$. The residuals are normalized by each target's standard
deviation. Each run receives equal total least-squares weight through a
per-sample factor $1/\sqrt{N_{\mathrm{run}}}$, and each run is capped at 3000
uniformly selected samples. Samples below $2\ \mathrm{m/s}$ are excluded.
The fitted parameters are stored in `nonlinear_bicycle_params.mat`; nominal
defaults are used when that file does not exist.

### Closed-loop simulation wrapper

`simulation_models/sim_nonlinear_bicycle.m` exposes the same identified model
as an eight-state global plant:

$$
X=\begin{bmatrix}x&y&\psi&v_x&v_y&r&\delta&\dot\delta\end{bmatrix}^{\mathsf T}.
$$

It calls `nonlinear_bicycle` for $y$, $v_y$, $\psi$, $r$, $\delta$, and
$\dot\delta$, so the tyre law, fitted parameter file, steering actuator, and
Euler discretization cannot drift away from the comparison model. The missing
global longitudinal kinematics are added as

$$
x_{k+1}=x_k+T_s\left(v_{x,k}\cos\psi_k-v_{y,k}\sin\psi_k\right).
$$

Longitudinal dynamics are outside the lateral-model scope. As in the other MPC
plants, the wrapper therefore sets $v_{x,k+1}$ to the next measured speed
provided by the evaluation scenario. Select it with

```matlab
plant_model = "nonlinear_bicycle";
```

The earlier `nonlinear_bicycle_linear_tire` plant remains available as a
separate legacy baseline; it uses linear cornering stiffness rather than the
nonlinear saturated tyre forces documented above.

## Nonlinear double-track model (`nonlinear_double_track`)

This model keeps separate front-left (FL), front-right (FR), rear-left (RL),
and rear-right (RR) tyre forces.

### Wheel velocities and slip angles

Yaw motion changes the longitudinal contact-patch velocity on each side:

$$
\begin{aligned}
v_{x,fl}&=\max(v_x-\tfrac{t_f}{2}r,v_{x,\min}), &
v_{x,fr}&=\max(v_x+\tfrac{t_f}{2}r,v_{x,\min}),\\
v_{x,rl}&=\max(v_x-\tfrac{t_r}{2}r,v_{x,\min}), &
v_{x,rr}&=\max(v_x+\tfrac{t_r}{2}r,v_{x,\min}).
\end{aligned}
$$

The slip angles are

$$
\begin{aligned}
\alpha_{fl}&=\delta-\operatorname{atan2}(v_y+l_fr,v_{x,fl}),\\
\alpha_{fr}&=\delta-\operatorname{atan2}(v_y+l_fr,v_{x,fr}),\\
\alpha_{rl}&=-\operatorname{atan2}(v_y-l_rr,v_{x,rl}),\\
\alpha_{rr}&=-\operatorname{atan2}(v_y-l_rr,v_{x,rr}).
\end{aligned}
$$

The nominal track widths are $t_f=t_r=1.20\ \mathrm m$ and the speed floor is
$v_{x,\min}=0.5\ \mathrm{m/s}$.

### Static axle loads and lateral load transfer

The static axle normal loads are

$$
F_{z,f}=mg\frac{l_r}{l_f+l_r},\qquad
F_{z,r}=mg\frac{l_f}{l_f+l_r}.
$$

For a current lateral-acceleration estimate $a_y$, the outside-minus-inside
load differences are

$$
\Delta F_{z,f}=\frac{2ma_yh}{t_f}\frac{l_r}{l_f+l_r},\qquad
\Delta F_{z,r}=\frac{2ma_yh}{t_r}\frac{l_f}{l_f+l_r}.
$$

Positive $a_y$ loads the right wheels in the code's sign convention:

$$
F_{z,fl}=\frac{F_{z,f}-\Delta F_{z,f}}{2},\qquad
F_{z,fr}=\frac{F_{z,f}+\Delta F_{z,f}}{2},
$$

with equivalent rear equations. The load differences are saturated before
splitting the axle load so each tyre remains above the configured minimum
normal load while the axle total is conserved.

### Load-sensitive tyre forces

For tyre $i$, the peak coefficient is

$$
D_i(F_{z,i})=\mu_iF_{z,i}
\left(\frac{F_{z,i}}{F_{z,i,\mathrm{nom}}}\right)^\lambda,
$$

where $\lambda=-0.10$ models tyre-load sensitivity. The lateral force is

$$
F_{y,i}=D_i(F_{z,i})\sin\!\left(C_i\arctan(B_i\alpha_i)\right).
$$

The front and rear body-frame lateral forces are

$$
F_{y,f}^{b}=(F_{y,fl}+F_{y,fr})\cos\delta,\qquad
F_{y,r}^{b}=F_{y,rl}+F_{y,rr}.
$$

Load transfer depends on lateral acceleration, while lateral acceleration
depends on the tyre forces. The implementation starts with $a_y=v_xr$ and
performs three fixed-point iterations of

$$
a_y=\frac{F_{y,f}^{b}+F_{y,r}^{b}}{m}.
$$

Finally,

$$
\dot v_y=a_y-v_xr,
$$

$$
\dot r=\frac{l_fF_{y,f}^{b}-l_rF_{y,r}^{b}+M_{t,f}}{I_z},
$$

where unequal steered front forces produce the geometric track-width moment

$$
M_{t,f}=\frac{t_f}{2}(F_{y,fl}-F_{y,fr})\sin\delta.
$$

This is not an externally commanded torque-vectoring moment; it is a moment
created by the four-wheel force geometry.

### Parameter identification

`nonlinear_double_track_training.m` fits

$$
\theta=\begin{bmatrix}B_f&B_r&\mu_f&\mu_r\end{bmatrix}
$$

using the same balanced, bounded one-step least-squares structure as the
nonlinear bicycle model. Track widths, centre-of-gravity height, load
sensitivity, $C_f$, and $C_r$ remain fixed at their nominal values. Fitted
parameters are stored in `nonlinear_double_track_params.mat`.

The double-track model adds left/right velocity differences, load transfer,
tyre-load sensitivity, and the front track moment. It still excludes roll and
pitch states, longitudinal tyre forces, combined slip, wheel-speed dynamics,
aerodynamics, and suspension dynamics.

## ANFIS models: common mathematical structure

All ANFIS variants use the same four scheduling features:

$$
z=\begin{bmatrix}v_y&r&v_x&\delta\end{bmatrix}^{\mathsf T}.
$$

The features are normalized using training-only statistics:

$$
z_n=(z-\mu_z)\oslash\sigma_z.
$$

At runtime each physical feature is clamped to its training range before
normalization. Therefore the models hold their boundary behavior outside the
observed domain instead of freely extrapolating.

Each output is a first-order Takagi-Sugeno ANFIS. For rule $i$,

$$
w_i(z_n)=\prod_{j=1}^{4}
\exp\!\left[-\frac{(z_{n,j}-c_{ij})^2}{2\sigma_{ij}^2}\right],
$$

$$
f_i(z_n)=a_i^{\mathsf T}z_n+b_i,
$$

and the output is

$$
\hat q(z_n)=\frac{\sum_iw_i f_i}{\sum_iw_i}.
$$

Separate ANFIS systems are trained for the $v_y$-related and $r$-related
outputs. Subtractive clustering initializes the fuzzy rules and MATLAB's
hybrid ANFIS optimizer tunes premise and consequent parameters.

### Local affine representation for MPC

`evalfis_mat` evaluates the normalized rule weights at the current operating
point and combines the rule consequents:

$$
\hat q\approx a_n^{\mathsf T}z_n+b_n,
\qquad
a_n=\sum_i\bar w_i a_i,
\qquad
b_n=\sum_i\bar w_i b_i.
$$

Transforming back to physical inputs gives

$$
a=a_n\oslash\sigma_z,qquad
b=b_n-a_n^{\mathsf T}(\mu_z\oslash\sigma_z),
$$

so locally $\hat q\approx a^{\mathsf T}z+b$. If a feature is clamped, its
local slope is explicitly set to zero and the affine offset is adjusted so the
model remains exact at the clamped operating point.

This is a frozen-rule-weight local affine model. It combines the consequent
slopes but does **not** include derivatives of the Gaussian firing strengths.
Consequently it is an MPC-compatible local approximation, not the full
Jacobian of the nonlinear fuzzy system.

The $y$, $\psi$, and steering states are supplied by the shared
physics/actuator equations; ANFIS only determines the $v_y$ and $r$ evolution.

## ANFIS increment model (`anfis_delta`)

The two training targets are the measured one-sample increments

$$
q_{v_y}=v_y(k+1)-v_y(k),\qquad
q_r=r(k+1)-r(k).
$$

For a runtime step $T_s$ and training sample time $T_{s,\mathrm{train}}$, the
implementation uses

$$
s_T=\frac{T_s}{T_{s,\mathrm{train}}},
$$

$$
v_y(k+1)=v_y(k)+s_T\hat q_{v_y}(z_k),
$$

$$
r(k+1)=r(k)+s_T\hat q_r(z_k).
$$

The affine versions of these equations are inserted into rows 2 and 4 of the
six-state $A$, $B$, and $C$ matrices. The direct function and matrix function
therefore execute the same locally affine update.

Its trainer:

- reads `datasets_training.mat` and `datasets_evaluation.mat` separately;
- rejects any rosbag path occurring in both splits;
- filters each signal with the configured Savitzky-Golay filter;
- keeps one transition in five;
- caps each training run at 1000 uniformly distributed transitions without
  duplicating short runs;
- computes normalization only from training runs;
- uses all held-out evaluation runs as ANFIS checking data; and
- saves the FIS from the epoch with the lowest held-out checking error.

The rule influence ranges are 0.35 for $\Delta v_y$ and 0.30 for $\Delta r$,
and both systems train for up to 200 epochs.

## ANFIS direct-next-state model (`anfis_direct`)

This formulation learns the absolute next values

$$
q_{v_y}=v_y(k+1),\qquad q_r=r(k+1).
$$

At the original training interval, the fuzzy outputs directly replace the two
lateral states. For a different runtime interval, the code interpolates from
the current state toward that learned next state:

$$
v_y(k+1)=(1-s_T)v_y(k)+s_T\hat q_{v_y}(z_k),
$$

$$
r(k+1)=(1-s_T)r(k)+s_T\hat q_r(z_k).
$$

This time scaling is an approximation: an absolute next-state map is tied more
strongly to its training sample time than a derivative model.

Its trainer uses the same independent real training/evaluation run split,
training-only normalization, filtering, one-in-five temporal downsampling,
1000-transition per-training-run cap, and held-out best-epoch selection as
`anfis_delta`.

## ANFIS derivative model (`anfis_dot`)

This formulation learns continuous-time-like derivatives:

$$
q_{v_y}=\dot v_y=a_y-v_xr,
$$

$$
q_r=\dot r\approx\frac{r(k+1)-r(k)}{T_{s,\mathrm{train}}}.
$$

Runtime propagation is

$$
v_y(k+1)=v_y(k)+T_s\hat q_{v_y}(z_k),
$$

$$
r(k+1)=r(k)+T_s\hat q_r(z_k).
$$

Because the output has derivative units, changing the runtime step has a clear
Euler-integration interpretation. However, the $\dot v_y$ target inherits the
quality, sign convention, synchronization, and noise of the measured lateral
acceleration.

Its trainer uses the same independent real training/evaluation run split and
balanced preprocessing as the other ANFIS models. The evaluation runs are used
only as ANFIS checking data and for the final model comparison.

## ANFIS LTV-residual model (`anfis_residuals`)

Let the LTV one-step prediction be

$$
x_{k+1}^{\mathrm{LTV}}=f_{\mathrm{LTV}}(x_k,U_k,T_{s,\mathrm{train}}).
$$

The two learned targets are

$$
e_{v_y}=v_y^{\mathrm{meas}}(k+1)-v_y^{\mathrm{LTV}}(k+1),
$$

$$
e_r=r^{\mathrm{meas}}(k+1)-r^{\mathrm{LTV}}(k+1).
$$

At runtime,

$$
v_y(k+1)=v_y^{\mathrm{LTV}}(k+1)+s_T\hat e_{v_y}(z_k),
$$

$$
r(k+1)=r^{\mathrm{LTV}}(k+1)+s_T\hat e_r(z_k).
$$

`anfis_residuals.m` evaluates the fuzzy residuals directly and adds them to a
direct call of the LTV predictor. `anfis_residuals_2.m` calls
`anfis_residuals_matrix.m`, which adds the frozen-rule affine residual
coefficients to rows 2 and 4 of the LTV matrices. The matrix version is the one
suited to the existing linear MPC construction.

This formulation retains the known small-slip structure and asks ANFIS to
learn only its systematic one-step error. Its runtime scaling assumes that a
one-step residual scales linearly with the interval, which is an approximation.

Its trainer uses the same independent real training/evaluation run split and
balanced preprocessing as the other ANFIS models. Residual targets are always
generated with the current steering-only `ltv` implementation, so changing the
LTV equations requires retraining this model.

## Evolving ellipsoidal TS model (`EEFIGLearning`, `TSGranule`)

The implementation follows the TS-EEFIG algorithm in [Alcala et al., 2022](https://doi.org/10.1016/j.asoc.2022.109698).
It learns local linear state-space consequents while allowing ellipsoidal
antecedent granules to evolve online. The two classes remain separated by role:
`TSGranule` owns one local rule and `EEFIGLearning` owns the granule set,
anomaly logic, auxiliary granule, and consequent-estimation policy.

For state $x_k\in\mathbb R^{n_x}$ and input
$u_k\in\mathbb R^{n_u}$, the premise/regression vector is

$$
\zeta_k=\begin{bmatrix}x_k^{\mathsf T}&u_k^{\mathsf T}\end{bmatrix}^{\mathsf T}.
$$

Rule $i$ has the consequent

$$
x_{k+1}^i=A_k^i x_k+B_k^i u_k,
$$

and the blended prediction is

$$
x_{k+1}=\sum_{i=1}^{N_G}g_k^i(\zeta_k)x_{k+1}^i,
\qquad
g_k^i=\frac{\xi_k^i}{\sum_q\xi_k^q}.
$$

The raw ellipsoidal membership is

$$
\xi_k^i(\zeta_k)=\exp\!\left(
-2\sqrt{\sum_j\left(
\frac{\zeta_{k,j}-\nu_{k,j}^i}
{\overline\zeta_{k,j}^i-\underline\zeta_{k,j}^i}
\right)^2}\right).
$$

A sample is admitted to granule $i$ when

$$
M_k^i=(\zeta_k-\nu_k^i)^{\mathsf T}
(\Omega_k^i)^{-1}(\zeta_k-\nu_k^i)\leq\epsilon,
$$

where $\epsilon$ is an inverse chi-square threshold. For an admitted sample,
the centre is tentatively updated using the accumulated-membership recursive
mean realization of equation (15),

$$
\nu_k^i=\nu_{k-1}^i+
\frac{g_{k-1}^i(\zeta_k)}{\alpha_{k-1}^i}
(\zeta_k-\nu_{k-1}^i).
$$

Here $\alpha_{k-1}^i$ is the accumulated membership evidence. This
denominator is required by the recursive EEFIG realization: without it, a
model containing one granule has $g=1$ and its centre would jump exactly to
every new sample.

The implementation stores the complete conditional dispersion matrices
$S_{k,j,l}^i$ and $T_{k,j,l}^i$, together with their lower/upper partition
counts. Their recursive updates implement equations (16)-(17):

$$
S_{k,j,l}^i=
\frac{(s_{k,j}-1)S_{k-1,j,l}^i+\nu_{k,l}^i-\zeta_{k,l}}
{s_{k,j}},
$$

$$
T_{k,j,l}^i=
\frac{(t_{k,j}-1)T_{k-1,j,l}^i+\zeta_{k,l}-\nu_{k,l}^i}
{t_{k,j}}.
$$

For the single bound vector used by the membership function, the code takes
the largest positive conditional spread in each dimension and applies

$$
\underline\zeta_k^i=
\max(\nu_k^i-\lambda S_k^i,\beta_S)
+\iota\left[\nu_k^i-
\max(\nu_k^i-\lambda S_k^i,\beta_S)\right],
$$

$$
\overline\zeta_k^i=
\min(\nu_k^i+\lambda T_k^i,\beta_T)
-\iota\left[
\min(\nu_k^i+\lambda T_k^i,\beta_T)-\nu_k^i\right].
$$

This per-dimension candidate selection is the only deliberate approximation
in the antecedent update: the complete iterative bound optimizer is delegated
by the MPC paper to its reference [39] and is not specified there. The code
does retain all cross-dimensional $S/T$ statistics and performs the stated PJG
accept-or-rollback test

$$
\mathcal I(G_k^i,\zeta_k)=M_k^i
\sum_{\zeta_j\in E_j^i}g_j^i(\zeta_j).
$$

The accumulated coverage term is corrected to first order for changes in the
lower bound, centre, and upper bound before the candidate index is evaluated.
This is essential because every changed granule also changes the normalization
of all memberships. The accepted index contributions are accumulated per
granule; a candidate with invalid coverage or a lower accumulated index is
rolled back atomically, including its centre, covariance, bounds, counts, and
membership moments.

The inverse covariance is updated with the rank-one recursion of equation
(18), using the accumulated first and second membership moments to construct
$\Gamma_k^i$ and $\Lambda_k^i$. Matrices are symmetrized and their eigenvalues
are floored only as a numerical safeguard. If the scalar recursion becomes
undefined for a degenerate initial configuration, the previous covariance is
retained rather than propagating non-finite values.

The auxiliary granule $G_k^{\mathrm{aux}}$ is the sample mean and covariance
of the current $\phi$-sample premise window. A new granule is created only
after more than $\bar n_a$ consecutive anomalies and, by default, when it is
$c$-separated from every existing granule:

$$
\lVert\nu_k^{\mathrm{aux}}-\nu_k^i\rVert
\ge c\sqrt{n_\zeta\max\left(
\lambda_{\max}(\Omega_k^{\mathrm{aux}}),
\lambda_{\max}(\Omega_k^i)\right)}.
$$

Offline training estimates the active consequent at every sample by windowed
least squares over the latest $\phi$ transitions. Online training uses the
paper's shared inverse-autocorrelation matrix $P_k$ and RLS update

$$
\Upsilon_k=\frac{P_k\zeta_k}
{\eta+\zeta_k^{\mathsf T}P_k\zeta_k},
\qquad
P_{k+1}=\eta^{-1}(I-\Upsilon_k\zeta_k^{\mathsf T})P_k,
$$

$$
\Theta_k^i=\Theta_{k-1}^i+
(x_{k+1}-\Theta_{k-1}^i\zeta_k)\Upsilon_k^{\mathsf T},
\qquad \Theta_k^i=\begin{bmatrix}A_k^i&B_k^i\end{bmatrix}.
$$

Granule creation/windowed LS and ordinary RLS are mutually exclusive during
one update. Defaults matching the paper's case study are
$\bar n_a=5$, $\eta=0.99$, $P_0=10^5I$, $\lambda=3$, and $\iota=0.3$.
Ellipsoidal admission uses the $99.9\%$ inverse chi-square threshold used by
the reference EEFIG implementation. The vehicle trainer uses $c=0.5$ for
granule separation. On the current real training/evaluation split, a sweep of
$c\in\{0.25,0.5,1.0\}$ produced respectively 21, 13, and 2 granules; $c=0.5$
was selected as the balance between held-out coverage and model complexity.
Because this project uses signed vehicle states, $\beta_S=-\infty$ by default;
set $\beta_S=0$ only when premise variables have first been mapped to a
non-negative domain.

### Lateral-vehicle realization

The vehicle model learns

$$
x_k^{\mathrm{lat}}=
\begin{bmatrix}v_y(k)&r(k)\end{bmatrix}^{\mathsf T},
\qquad
u_k^{\mathrm{lat}}=
\begin{bmatrix}v_x(k)&\delta(k)\end{bmatrix}^{\mathsf T},
$$

with direct next-state target

$$
x_{k+1}^{\mathrm{lat}}=
\begin{bmatrix}v_y(k+1)&r(k+1)\end{bmatrix}^{\mathsf T}.
$$

`prepare_eefig_data.m` constructs transitions independently inside each run,
so no target ever connects the end of one rosbag to the beginning of another.
`startNewRun()` clears the moving regression window, auxiliary granule, and
anomaly state while preserving all learned granules and consequents. Offline
training uses every selected training transition in dataset order. The held-out
evaluation split is used only for frozen predictions and diagnostics.

Training-only scale factors are applied without subtracting a mean:

$$
x_n=D_x^{-1}x^{\mathrm{lat}},\qquad
u_n=D_u^{-1}u^{\mathrm{lat}}.
$$

Scale-only normalization preserves the origin because the local consequents
do not have an affine intercept. If a local normalized model is

$$
x_{n,k+1}=A_nx_{n,k}+B_nu_{n,k},
$$

its physical-coordinate matrices are

$$
A_{\mathrm{lat}}=D_xA_nD_x^{-1},\qquad
B_{\mathrm{lat}}=D_xB_nD_u^{-1}.
$$

The learned model is discrete at the dataset sampling interval
$T_{s,\mathrm{train}}$. For a runtime interval $T_s$, the wrapper uses

$$
s_T=\frac{T_s}{T_{s,\mathrm{train}}},\qquad
A_{\mathrm{lat}}(T_s)=I+s_T
(A_{\mathrm{lat}}(T_{s,\mathrm{train}})-I),
$$

$$
B_{\mathrm{lat}}(T_s)=s_TB_{\mathrm{lat}}(T_{s,\mathrm{train}}).
$$

In the six-state affine model returned by `eefig_matrix.m`, the two learned
rows are embedded as

$$
\begin{bmatrix}v_y(k+1)\\r(k+1)\end{bmatrix}
=A_{\mathrm{lat}}
\begin{bmatrix}v_y(k)\\r(k)\end{bmatrix}
+b_{v_x}v_x(k)+b_\delta\delta(k).
$$

Thus, $b_\delta$ is inserted in column 5 of the six-state $A$ matrix and
$b_{v_x}v_x$ becomes the lateral part of the affine vector $C$. Neither
$v_x$ nor measured $\delta$ is treated as an MPC decision. The only decision
input remains requested steering $u_\delta$, acting through the shared
second-order steering actuator. Kinematic rows for $y$ and $\psi$ are the same
as in the other models.

`eefig.m` is the frozen model registered in `compare_models.m`.
`eefig_online_update.m` is deliberately separate and must be called only after
the next measured state is available. `eefig_reset.m` discards all evaluation-
time adaptation and reloads the saved offline artifact. The script
`eefig_adaptation_analysis.m` evaluates both frozen and predict-then-update
one-step behavior and both frozen and continuously adaptive propagation,
reloading the identical offline baseline before each independent run.

The generic learner retains the paper's shared-$P_k$ RLS as its default. The
trained vehicle model explicitly uses one $P_k^i$ per granule with
$P_0^i=10I$. On the current raw held-out runs, shared $P_0=10^5I$ generated
large switching transients, whereas per-granule $P_0=10I$ improved adaptive
RMSE over the frozen model for both learned states. This project-specific
choice changes only online consequent adaptation; antecedent evolution and the
offline model remain unchanged.

## Comparison and interpretation notes

- The model APIs retain steering-actuator states for eventual MPC use, but the
  primary identification comparison isolates vehicle dynamics.
  `compare_models.m` imposes measured $\delta(k)$ at every propagation step
  and places the actuator subsystem at equilibrium. Requested steering and
  predicted steering states therefore cannot contaminate the primary
  $v_y/r/\psi$ ranking. A separate end-to-end diagnostic starts from measured
  $(\delta,\dot\delta)$ and propagates requested steering through each model's
  actuator; it is reported separately and is not a vehicle-dynamics result.
- Measured body yaw is read from `/as/c/state.odom.heading`, projected to a
  wrapped planar angle, unwrapped before interpolation, and used for $\psi$.
  Trajectory course computed from SLAM $x/y$ is not used as vehicle heading,
  and unreliable SLAM position is not scored in the dynamics comparison.
- One-step predictions use every valid held-out transition. Recursive tests
  use identical rolling origins for every model and the horizons
  $\{1,5,10,20,40,60\}$ samples. At $T_s=0.02$ s the emphasized 60-step MPC
  horizon is 1.2 s. Only origins having the full 60 future samples are used,
  so horizon curves are paired and changes with horizon are not caused by a
  changing sample population.
- The comparison consumes the resampled signals stored in the evaluation MAT
  file and does not reapply the centred Savitzky--Golay training filter. This
  measures performance against the deployable held-out signal stream and,
  importantly, prevents future samples entering the causal adaptive EEFig
  result through non-causal smoothing. Training/validation losses printed by
  individual trainers use their own documented filtered data and therefore
  should not be compared numerically with these raw-stream scores.
- The persistence baseline keeps $v_y$ and $r$ constant and integrates heading
  at the initial constant yaw rate. Positive skill relative to this baseline
  means the model improves RMSE; negative skill means persistence is better.
- Sample-weighted RMSE gives every prediction window equal influence;
  run-weighted RMSE first computes each run's RMSE and then gives all runs
  equal influence. The declared-lap-weighted number weights each run-level
  RMSE by its metadata lap count. It is not a true per-lap metric because the
  stored datasets do not contain lap boundaries.
- Adaptive EEFig is evaluated causally: predict and score first, then update
  from the newly arrived measured transition. Each held-out run reloads the
  same offline model and clears transient state. At a rolling origin, the
  current adapted model is frozen throughout the candidate future rollout;
  no future measurements are used inside the prediction horizon.
- Recursive errors above 10 m/s in $v_y$ or 5 rad/s in $r$ are flagged as
  numerical divergence. These thresholds do not clamp predictions, discard
  windows, or modify reported metrics. Figures alone use a robust normal-scale
  axis when a value exceeds ten times the median plotted value; the annotation
  reports how many values are off scale and the tables retain the full values.
- The comparison prints a speed sweep of the LTV lateral-state spectral radius.
  The unclamped forward-Euler system at $T_s=0.02$ s becomes unstable below
  approximately 3 m/s. The current first-stage mitigation clamps the lateral
  scheduling speed to 3 m/s, while leaving outputs unconstrained so any
  remaining divergence stays visible. `anfis_residuals` uses this same LTV
  base. Because changing the base changes its residual definition, that ANFIS
  model should be retrained before its final thesis comparison.
- The nonlinear bicycle and double-track models are propagation models, but
  they do not currently expose an analytic local linearization for MPC.
- `ltv`, `anfis_delta`, `anfis_direct`, `anfis_dot`, `eefig`, and the matrix
  residual implementation expose $A$, $B$, and $C$ in the form required by the
  present linear MPC.
- ANFIS models must be retrained after changing their feature vector, training
  targets, sample time, preprocessing, or dataset split. Their generated `.mat`
  files are part of the model definition even though the source equations are
  in `.m` files.
- One-step accuracy and propagated-state accuracy answer different questions.
  A model can achieve low one-step error while accumulating bias or becoming
  unstable during recursive propagation.
- Comparisons must use complete held-out runs. Randomly splitting adjacent
  samples from the same run causes temporal leakage and produces overly
  optimistic validation errors.
- The dataset `Mz` audit may remain in the data-preparation tools as a safeguard,
  but $M_z$ is not a model feature or control input.
