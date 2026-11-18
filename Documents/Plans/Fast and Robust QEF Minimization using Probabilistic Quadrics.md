# Fast and Robust QEF Minimization using Probabilistic Quadrics

Plan written 2026-11-18 for adding the QEF solve of *Fast and Robust QEF Minimization using Probabilistic Quadrics* (Trettner and Kobbelt, 2020; `Documents/Papers/Fast and Robust QEF Minimization using Probabilistic Quadrics.md`) as a third QEF solver, beside Simple and SVD, in the managed, Burst and GPU implementations.

**Status:** draft. Nothing is built. The user decided four things on 2026-11-18:
1. **A third solver,** not a replacement for SVD.
2. **Managed, Burst and GPU.**
3. **The default noise level is mine to choose.** It is 0.05 (finding 5).
4. **Floats for now.** The sums of a QEF stay in floats, in octrees too.

The smaller decisions at the end are open. Step 0 waits for the go-ahead.

The feasibility study behind the plan was done on 2026-11-18 with an experiment outside the repo. Its numbers are under Measurements.

## Goal

Every voxelizer's **QEF Solver** setting has a third value, **Probabilistic**, with one setting of its own, **Normal Deviation**. It works in every algorithm and every implementation, the GPU's included, and in the interactive editor. A voxelizer set to Simple or SVD gives the mesh it gives today, bit for bit, and SVD stays the default.

## What the paper does

- **The idea (sections 1 and 3.4).** A classical quadric takes its planes as exact. The paper takes each plane as a sample of a distribution and minimizes the expected squared error. The expected error is again a quadric, the *probabilistic quadric*, so everything built on quadrics takes it unchanged.
- **Planes (section 3.1).** For a plane through `q` with normal `n`, with Gaussian noise `Σ_n` in the normal: `A = n nᵀ + Σ_n` and `b = n nᵀ q + Σ_n q`. That is the classical plane quadric plus a pull toward the sample's own point, weighted by the noise. Noise in the position changes the error's constant and not the minimizer.
- **The solve (sections 3.3 and 4.1).** With any noise above zero, `A` has full rank, so the minimizer is `A⁻¹ b`: a 3 × 3 linear system with no SVD, no truncation and no iterations. The paper measures it at over 50 times the speed of an SVD.
- **Dual contouring (section 5.1, figure 3).** One noise level `σ_n` for all normals. At 0 it is the classical QEF. Toward infinity the vertex goes to the mass point of the samples. In between, features are kept and noise is not amplified.
- **Not used here:** the triangle quadric (section 3.2), decimation (4.2), smoothing (5.2) and subdivision (5.3). See Out of scope.
- **The authors' code** is one header, MIT licensed (github.com/Philip-Trettner/probabilistic-quadrics). I read it. Its `minimizer()` is an unrolled LDLᵀ solve of about 25 lines.

## What exists today

- **Two solvers,** picked by `QEFSolver.Type { Simple, SVD }` on every voxelizer, SVD by default.
  - **Managed:** `SimpleQEF` and `SVDQEF` behind the `QEFSolver` interface, made by a `switch` in each of the three managed implementations.
  - **Burst:** one struct, `BurstQEF`, with a branch on its type. It is made in five places with `BurstQEF.create( type, minimizerIterations )`, from `BurstSettings` or, in Occupancy, from the job's own fields.
  - **GPU:** `QEF.hlsl` has the SVD solver only. Uniform's driver is handed the solver type and ignores it, Occupancy's is not handed it, and the voxelizer's inspector turns the solver into SVD whenever the implementation is GPU.
- **What a QEF keeps:** `AᵀA`, `Aᵀb`, `bᵀb`, the sum of the intersections and their count. The SVD solver also keeps the dimension of the feature its mass point lies on, and when two QEFs are combined it keeps the mass point of the sharper feature only.
- **The SVD solve:** Jacobi sweeps, **Minimizer Iterations** of them, then a pseudo inverse that drops eigenvalues under 0.1, about the mass point.
- **The error** is the classical QEF at the vertex that was placed, `xᵀ AᵀA x − 2 xᵀ Aᵀb + bᵀb`, in floats. Adaptive and Manifold compare it with their thresholds.
- **Cells with several pieces of contour** get the mass point and not the minimizer in Uniform and Adaptive (`placeVertex`).
- **Interactive editor:** `select` carries the solver from one algorithm to the next, and the contour's signature takes it in.
- **Scenes:** an `SVD Solve` scene for Uniform, Adaptive and Manifold, Managed and Burst each.

### Findings that shape the work

1. **The solver needs no new state.** With one `σ` for every sample, the probabilistic quadric of a cell follows from the sums a QEF already keeps, with `N` the number of samples and `p` their mass point:

   `x = p + ( AᵀA + N σ² I )⁻¹ ( Aᵀb − AᵀA p )`

   `add` and `combine` stay as they are, and so does the GPU's struct. Only the solve is new. The mass point has to be that of every sample, as Simple has it, not the SVD solver's mass point of the sharpest feature.

2. **The solve is small and has nothing to iterate.** It is the authors' unrolled LDLᵀ, for the move from the mass point, which keeps the numbers small. **Minimizer Iterations** means nothing to this solver.

3. **`σ` has to be above zero.** At zero the matrix is the classical one, which is singular on every flat face. The setting needs a lower bound (finding 8).

4. **In single cells it matches SVD on clean data and keeps creases that SVD flattens** (Measurements, single cells). The SVD solver drops eigenvalues under 0.1, which in a cell with a handful of samples is any crease whose normals are less than about 20° apart, and any edge that only one sample of the cell sees. At `σ` = 0.05 a ridge with normals 10° apart is 0.10 cells from where it should be, against 0.32 for SVD, and the worst vertex on a box's edges is 0.012 cells off against 0.087.

5. **The default is 0.05,** from the single-cell measurements at 0.01 to 0.25:
   - **Lower values** are sharper on shallow creases and worse at everything else: vertices on a ball wander along the surface (finding 9), noisy normals throw vertices out of their cells (finding 6) and float rounding grows (finding 8).
   - **Higher values** round every feature toward the mass point. At 0.1 a box's corners are 0.02 cells in, twice SVD's.
   - **At 0.05** corners and edges of boxes are within 0.002 cells of SVD's, shallow creases are well ahead of it, and the float error is 0.005 cells at 256³.
   - **Step 2 checks it on the project's own models** and changes it if they disagree.

6. **`σ` has to be near the noise in the normals.** With normals off by about 3°, `σ` = 0.01 puts 53% of a ball's vertices outside their cells, by up to 1.6 cells. At 0.05 it is 8%, at 0.1 it is 1.5%, and SVD has 0.2%. The project's normals are central differences of exact shapes, good to a tenth of a degree, so this matters for noise volumes and little else. It is what the setting is for.

7. **In summed QEFs it pulls vertices off features, and simplifies less than SVD.** Adaptive simplification and Manifold clustering sum the QEFs of many cells. The pull toward the mass point weighs every sample alike, so a feature that few of a block's samples see gives way to the many that do not.
   - **A box turned off the axes, blocks of 16 cells, `σ` = 0.05:** vertices are 0.84 cells from the edge or corner on average, against 0.0015 for SVD, and 32% of blocks have an error under 1e-5, against 95%.
   - **A box along the axes** is unaffected up to 0.05: every block stays under 1e-5.
   - **It stays safe:** the pull raises the error, the error is what the threshold tests, and a block that fails is not collapsed. The mesh is not wrong, it has more vertices than SVD's.
   - **What follows:** this solver suits Uniform and Occupancy-Based Dual Contouring. In Adaptive and Manifold it works, and SVD stays the better choice where simplification is wanted. The README says so.
   - **It does not help a cube reach 8 vertices.** That is rounding in the error and clustering bound to cells, and this solver adds error at coarse levels.

8. **Floats set the lower bound of `σ`.** The sums are kept about the world origin, and the solve divides what is left of them after cancellation by `N σ²`.
   - **Single cells,** worst vertex against a solve in doubles, in cells:

     | Grid | `σ` 0.1 | 0.05 | 0.03 | 0.02 | 0.01 |
     |---|---|---|---|---|---|
     | 64³ | 0.0003 | 0.0011 | 0.0029 | 0.0060 | 0.024 |
     | 128³ | 0.0006 | 0.0023 | 0.0063 | 0.014 | 0.056 |
     | 256³ | 0.0013 | 0.0053 | 0.014 | 0.032 | 0.13 |

   - **The setting's range is 0.02 to 1,** so that the worst case at 256³ is 0.03 cells.
   - **Blocks of 16 cells at 64³** are off by 0.013 cells at 0.03 and 0.11 at 0.01. Decision 4 accepts that.
   - **Managed and Burst will differ by about this much too.** Mono keeps the intermediate results of an expression at a higher precision than Burst, the sums differ in their last bits, and the solve magnifies that by `1 / σ²`. `BurstQEFTest` allows 1e-5 relative and the contour comparisons 5e-3 cells. Step 1 measures what this solver needs and the tolerances for it are set there.
   - **A way out that stays in floats,** not planned: sums about the centre of the cell are 25 times closer in single cells. It would change what `add` is given, for this solver only.

9. **It does not steady a moving shape.** I expected the hard 0.1 cut of the SVD solver to make vertices jump and this solver to be smoother. Measured, the large jumps are the same for both: they come from the mass point, when an edge starts or stops crossing the contour. At small `σ` it is worse: on a ball, vertices move 0.023 cells for each hundredth of a cell the ball moves at 0.05, against 0.011 for SVD and 0.041 at 0.03.

10. **The solve is about five times faster, and the contour will hardly notice.** Solve and error took 25 ns against 128 ns for SVD and 88 ns for Simple, under the .NET JIT and not Burst. A contour solves once per cell with a vertex and samples the volumes about a hundred times for the same cell. I expect a few percent at most. Step 5 measures it in Burst and on the GPU.

11. **The error stays the classical one.** The paper's own error adds `σ²` times the squared distance to every sample. It grows with the size of a block whether the block is flat or not: on a box along the axes, 100% of 16-cell blocks pass a threshold of 2e-4 with the classical error and 11% with the paper's at `σ` = 0.05. The thresholds of the scenes and tests keep their meaning with the classical one.

12. **On the GPU the solver type has to arrive first.** The shaders get the type and the deviation in their configuration, and `QEF.hlsl` branches on the type. Simple stays unavailable there, and the inspector's rule becomes "Simple turns into SVD on the GPU".

13. **Forty-one tests take the new solver by themselves.** They are parametrized with `[Values] QEFSolverType`, in eight fixtures: the comparisons of Burst with Managed, the topology tests of Uniform, Adaptive and Manifold, and the tests of `placeVertex`.
    - **The managed halves are slow,** so the suite gets longer. Step 2 measures by how much, and no single test may pass two minutes.
    - **Some may not hold for this solver as written.** `clusteringGoesAsFarAsTheManifoldCriterionAllows` and the comparisons under simplification decide at a threshold, where finding 8's differences can tip a collapse. Each one that fails is looked at and either holds with a tolerance of its own or is listed here with the reason.
    - **The `Start` hash probe** names its solvers, so its 480 configurations are untouched and stay the check that nothing else changed. It gains lines for the new solver.

14. **The enum is saved as a number.** `Probabilistic` goes after `SVD`, so every saved scene keeps its solver.

## Design

### The setting

- `QEFSolver.Type { Simple, SVD, Probabilistic }`.
- `Voxelizer.normalDeviation`, a float from 0.02 to 1, default 0.05, labelled **Normal Deviation**. Tooltip: how uncertain the normals are taken to be, as a fraction of their length; higher values pull vertices toward the average of their cell's samples.
- **The inspector** shows Normal Deviation only while the solver is Probabilistic, and hides Minimizer Iterations then (decision 2).
- **New components** still start with SVD.

### The solve

For a QEF with `N` samples, sums `AᵀA` and `Aᵀb`, and mass point `p`:

1. `r = Aᵀb − AᵀA p`.
2. `M = AᵀA + N σ² I`.
3. Solve `M d = r` by the unrolled LDLᵀ of the authors' header, without its fused multiply-adds, which Mono does not have.
4. The vertex is `p + d`. Its error is the classical QEF there.

The mass point and its error (`solveMassPoint`) are as Simple has them. `placeVertex` is unchanged: a cell with several pieces of contour gets the mass point.

### Managed

`Assets/Scripts/Managed/ProbabilisticQEF.cs`, a `QEFSolver`. It keeps the sums as `SVDQEF` does, uses its `SymmetricMatrix3x3` and `SVD.calculateError`, and combines by adding everything. The interface's `minimizerIterations` is there and unused. The three managed implementations make it in their `switch`.

### Burst

- `BurstQEF` gains a `normalDeviation` field and a third branch in `solve`. `combine` and `solveMassPoint` treat the new type as they treat Simple.
- `BurstQEF.create` takes the deviation, in its five callers.
- `BurstSettings` and Occupancy's job carry it.

### GPU

- `QEF.hlsl`: the LDLᵀ solve, and `solveQEF` takes the solver type and the deviation.
- `UniformDualContouring.compute` and `OccupancyDualContouring.compute`: two more fields in `Configuration`, on both sides, field for field.
- The drivers pass them. Occupancy's driver is handed the solver type for the first time.
- A voxelizer set to Simple on the GPU contours with SVD, as today.

### Interactive editor

- `select` carries `normalDeviation` with the solver.
- The signature takes the deviation in while the solver is Probabilistic, and not otherwise, as it does the crease angle.

### Example scenes

A `Probabilistic Solve` scene beside each of the six `SVD Solve` scenes, made from them with the solver switched (decision 5). Scene files are made and changed only with the user's go-ahead at that step.

## Tests

Each new test is shown to fail on a deliberate mistake, as in the earlier plans.

- **The solve** (new `ProbabilisticQEFTest`, managed):
  - **A flat face:** the vertex is the mass point, at any deviation.
  - **Three faces at right angles** along the axes, with `N_k` samples on face `k` at `d_k`: the vertex is `p_k + ( d_k − p_k ) N_k / ( N_k + N σ² )` on each axis, which is the solve in closed form.
  - **The two ends:** at 0.02 a well-sampled corner is within a small distance of SVD's, and at 1 the vertex is most of the way to the mass point, each against the closed form.
  - **An edge:** the vertex is on the line of the edge, at the mass point's place along it. Nothing is infinite or not a number, for one sample, for samples in a plane and for samples on a line.
  - **A shallow ridge,** normals 10° apart: the vertex is nearer the ridge than SVD's.
  - **Moved samples move the vertex with them:** the same samples far from the origin give the same vertex relative to them, within the float bound of finding 8. This catches a solve about the origin.
  - **The error** is equation 1 of the dual contouring papers at the vertex.
  - **Combined QEFs** solve as the QEF of all their samples, with the mass point of all of them.
- **Burst against Managed:** `BurstQEFTest` takes the new type, with the tolerance step 1 measures.
- **Contours:** the 41 parametrized tests (finding 13). A cube and a turned cube in Uniform have a vertex within a tolerance of every corner, in Managed, Burst and on the GPU.
- **GPU:**
  - the solve kernel against the managed solve, in the play mode QEF tests beside `SVDTest`;
  - Uniform's GPU contour against the CPU's and Occupancy's against Burst's, with the new solver;
  - a voxelizer set to Simple on the GPU still contours with SVD.
- **Settings:** `VoxelizerDefaultsTest` holds SVD as the default solver and 0.05 as the deviation. The deviation changes a contour under Probabilistic and nothing under SVD or Simple.
- **Interactive editor:** the carry-over tests take the deviation in. The signature changes with it under Probabilistic and not under SVD.
- **No change for the others:** the 480 `Start` hashes of step 0 after every step.
- **Time:** no test over two minutes. The suite's time is recorded before and after step 2.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts. Nothing is committed for you.

0. **Baseline (small).** Both suites green. The `Start` hashes and the Burst and GPU timings recorded with Unity in front.
1. **The solve, managed and Burst (small).** `ProbabilisticQEF`, and the solve in `BurstQEF` as a function the tests can call, without the enum value yet. `ProbabilisticQEFTest`, and the Burst solve against the managed one. **Measured here:** how far Managed and Burst are apart at each deviation, which sets the tolerances (finding 8).
2. **The solver in the CPU implementations (medium).** The enum value, the setting and its inspector, the managed `switch`es, `BurstSettings` and the five `create` calls. The 41 tests take it in. **Measured here,** on the project's models and shapes, against SVD:
   - distance of vertices from single shapes and from the edges and corners of cubes, upright and turned;
   - vertices outside their cells;
   - vertex counts of Adaptive and Manifold at the scenes' thresholds;
   - the suite's time.

   The default of 0.05 is confirmed or changed here, and the questions under "Asked after step 2" are put.
3. **GPU (medium).** `QEF.hlsl`, both shaders' configurations and drivers, the inspector's rule, the GPU tests.
4. **Interactive editor and scenes (small).** `select`, the signature, the editor tests. The six scenes, with the user's go-ahead.
5. **Timings and documentation (small).** Burst and GPU timings of the three solvers at 64³ and 128³, with Unity in front. README: the solver in the implementations section, what it is good and bad at (findings 4, 6 and 7), the setting, the paper and the MIT credit. `AGENTS.md`: the paper among the algorithms.

## Measurements

All from the feasibility experiment of 2026-11-18, not from the Unity pipeline. It is a console program that compiles `BurstQEF.cs` unmodified and samples exact shapes as the project does: a 64³ grid one unit wide, six bisection steps, normals by central differences of 1e-4, all in floats. It is kept outside the repo, in the session's probes folder under `probabilistic-quadrics`. Distances are in cells.

### Single cells, clean data

Distance from the vertex to the true edge or corner, mean / worst.

| Case | SVD | `σ` 0.02 | 0.03 | 0.05 | 0.07 | 0.1 |
|---|---|---|---|---|---|---|
| Box along the axes, edges | 0.007 / 0.087 | 0.007 / 0.011 | 0.007 / 0.011 | 0.007 / 0.012 | 0.007 / 0.013 | 0.009 / 0.017 |
| Box along the axes, corners | 0.010 / 0.012 | 0.010 / 0.012 | 0.010 / 0.012 | 0.011 / 0.014 | 0.014 / 0.018 | 0.021 / 0.027 |
| Turned box, edges | 0.003 / 0.031 | 0.003 / 0.023 | 0.004 / 0.022 | 0.005 / 0.020 | 0.007 / 0.024 | 0.012 / 0.031 |
| Turned box, corners | 0.005 / 0.008 | 0.005 / 0.008 | 0.005 / 0.008 | 0.007 / 0.011 | 0.012 / 0.016 | 0.022 / 0.030 |
| Ridge, normals 30° apart | 0.012 / 0.72 | 0.007 / 0.020 | 0.009 / 0.032 | 0.017 / 0.099 | 0.030 / 0.18 | 0.056 / 0.30 |
| Ridge, normals 10° apart | 0.32 / 0.78 | not run | 0.046 / 0.12 | 0.098 / 0.25 | 0.15 / 0.38 | 0.20 / 0.51 |

On a ball of radius 0.3, vertices are 0.010 cells from the surface on average with every solver.

### Single cells, normals off by about 3°

Gaussian noise of 0.05 on each component of every normal. Share of vertices outside their cells, and how far the worst one is.

| Case | SVD | `σ` 0.01 | 0.03 | 0.05 | 0.07 | 0.1 |
|---|---|---|---|---|---|---|
| Ball | 0.2%, 0.02 | 53%, 1.6 | 22%, 0.46 | 7.8%, 0.23 | 3.6%, 0.14 | 1.5%, 0.05 |
| Turned box | 2.2%, 0.45 | 60%, 2.0 | 24%, 0.82 | 9.9%, 0.50 | 5.8%, 0.47 | 3.4%, 0.45 |

The mean distance from the ball's surface is 0.014 for SVD and 0.013 from 0.03 upward.

### Summed QEFs

Blocks of cells of a box turned 20°, 30° and 10° about the axes, each summed into one QEF. Share of blocks whose classical error at the vertex is under 1e-5, and the mean distance from the vertex to the block's edge or corner.

| Block | SVD | `σ` 0.02 | 0.05 | 0.1 |
|---|---|---|---|---|
| 4 cells | 100%, 0.002 | 100%, 0.008 | 98%, 0.041 | 88%, 0.13 |
| 8 cells | 98%, 0.003 | 94%, 0.026 | 69%, 0.13 | 57%, 0.37 |
| 16 cells | 95%, 0.0015 | 54%, 0.29 | 32%, 0.84 | 30%, 1.5 |

For a box along the axes every block is under 1e-5 up to `σ` = 0.05, and vertices are within 0.02 cells of their feature.

### A moving shape

Each shape moved by a hundredth of a cell 150 times. How far a cell's vertex moves per step, on average.

| Shape | SVD | `σ` 0.03 | 0.05 | 0.1 |
|---|---|---|---|---|
| Ball | 0.011 | 0.041 | 0.023 | 0.013 |
| Turned box | 0.013 | 0.013 | 0.013 | 0.013 |

Steps over half a cell on the turned box: 780 for SVD and 649 at 0.05, of 1.3 million.

### The solve

Solve and error of one QEF of a single cell, under the .NET JIT: SVD 128 ns, Simple 88 ns, probabilistic 25 ns.

## Out of scope

- **Replacing the SVD solver,** or changing any default. Decided on 2026-11-18.
- **Sums in doubles or about a local origin.** Decided on 2026-11-18 (finding 8).
- **The triangle quadric** (section 3.2) and the mesh simplifier. `BurstMeshSimplifier` already settles flat regions with line quadrics. The paper's triangle quadric is another way to do that and would be a plan of its own.
- **Noise that differs from sample to sample or by direction.** One `σ` for a contour is what lets the solver use the sums as they are (finding 1). Anything else needs a matrix more in every QEF.
- **Noise in the positions.** It does not move the vertex.
- **The paper's expected error** as the measure for simplification and clustering (finding 11).
- **The flat faces of Manifold Dual Contouring** (finding 7).
- **Quadric smoothing and subdivision** (sections 5.2 and 5.3).

## Decisions

1. **Name and range of the setting.** Recommended: **Normal Deviation**, 0.02 to 1, default 0.05 (findings 5 and 8). The paper calls it `σ_n` and gives it in percent of the normal's length; 0.05 is its 5%.
2. **What the inspector shows.** Recommended: Normal Deviation only under Probabilistic, and Minimizer Iterations hidden then, since neither means anything to the other solvers. The other way shows both always, which is simpler and leaves a dead setting in view.
3. **The error for simplification and clustering.** Recommended: the classical QEF at the placed vertex (finding 11).
4. **Port the authors' solve?** Recommended: yes, the 25 lines of `minimizer()`, credited in a comment and in the README. The licence is MIT, as this project's is.
5. **Example scenes.** Recommended: six `Probabilistic Solve` scenes beside the `SVD Solve` ones. The other way is none, since the interactive editor shows the solver on any model.
6. **Editor and README in this plan?** Recommended: yes, as steps 4 and 5.

### Asked after step 2

- **The default,** if the project's models disagree with 0.05.
- **Tests that do not hold for this solver as written** (finding 13): which get a tolerance of their own and which leave the solver out.
- **Whether Adaptive and Manifold should warn** in the inspector that this solver simplifies less than SVD.
