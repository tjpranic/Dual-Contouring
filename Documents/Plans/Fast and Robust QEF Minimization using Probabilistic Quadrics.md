# Fast and Robust QEF Minimization using Probabilistic Quadrics

Plan written 2026-11-18 for adding the QEF solve of *Fast and Robust QEF Minimization using Probabilistic Quadrics* (Trettner and Kobbelt, 2020; `Documents/Papers/Fast and Robust QEF Minimization using Probabilistic Quadrics.md`) as a third QEF solver, beside Simple and SVD, in the managed, Burst and GPU implementations.

**Status:** in progress. Steps 0 to 2 were done on 2026-11-18: every algorithm on the CPU, managed and Burst, has the solver. Step 3, the GPU, was done the same day: every implementation has the solver now. Step 4 waits for review, and it needs the user's go-ahead for the scenes. The user decided four things on 2026-11-18:
1. **A third solver,** not a replacement for SVD.
2. **Managed, Burst and GPU.**
3. **The default noise level is mine to choose.** It is 0.05 (finding 5).
4. **Floats for now.** The sums of a QEF stay in floats, in octrees too.

The user answered the decisions at the end on 2026-11-18 and gave the go-ahead: "1. The setting should be called Probabilistic. You can name anything else. 2 - 6. As you recommend." What decision 1 names is asked under it.

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
   - **Managed and Burst will differ by about this much too.** Mono keeps the intermediate results of an expression at a higher precision than Burst, the sums differ in their last bits, and the solve magnifies that by `1 / σ²`. `BurstQEFTest` allows 1e-5 relative and the contour comparisons 5e-3 cells. Step 1 measured about 3.1e-8 / σ² between the two on `BurstQEFTest`'s samples, 7.8e-5 at 0.02 (step 1).
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
  - **Moved samples move the vertex with them:** the same samples far from the origin give the same vertex relative to them, within the float bound of finding 8. This catches a pull toward the origin in place of the mass point.
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

0. **Baseline (small).** Both suites green. The `Start` hashes and the Burst and GPU timings recorded with Unity in front. **Done 2026-11-18:**
   - **Suites:** edit mode 3781 of 3781 in 457 s, play mode 21 of 21.
   - **`Start` hashes:** all 480 identical to those at the end of the remeshing plan's step 4. These are the baseline from here on.
   - **Timings:** under Measurements, Baseline timings. Simple and SVD take the same time within the noise in every algorithm: the solve is too small a part of a contour to show.
   - **Tooling,** kept in the probes folder outside the repo: `QEFTimingProbe.cs` and `run_qef_timing.sh` for the timings, `mutations_pq_step1.py` for step 1's deliberate mistakes.
1. **The solve, managed and Burst (small).** `ProbabilisticQEF`, and the solve in `BurstQEF` as a function the tests can call, without the enum value yet. `ProbabilisticQEFTest`, and the Burst solve against the managed one. **Measured here:** how far Managed and Burst are apart at each deviation, which sets the tolerances (finding 8). **Done 2026-11-18:**
   - **Built:**
     - `Assets/Scripts/Managed/ProbabilisticQEF.cs`, a `QEFSolver`. It keeps the sums as `SVDQEF` does, and the mass point of every sample. `ProbabilisticQEF.minimize` is the authors' `minimizer()`, credited in a comment. Its constructor takes the deviation, and `minimizerIterations` is there for the interface only.
     - `BurstQEF.solveProbabilistic( normalDeviation, ... )` and the private `minimizeProbabilistic`, the same operations in the same order.
     - Nothing outside the tests calls either yet, so every contour is as it was.
   - **Tests:**
     - `ProbabilisticQEFTest`, 38 cases:
       - the closed form for a face, an edge and a corner, square to the axes and turned, at 0.02, 0.05, 0.25 and 1;
       - four degenerate sets of samples that must give the mass point: one sample, no normals, a slab, and planes turning about one line;
       - the shallow ridge against `SVDQEF`;
       - the moved samples;
       - the error at the vertex;
       - combined QEFs.
     - **A correction to the plan:** it said the moved samples catch a solve about the origin, which was wrong. A solve about the origin gives the same vertex, within rounding. What the test catches is a pull toward the origin in place of the mass point.
     - `BurstQEFTest.probabilisticQEFsSolveAsTheManagedOnesDo`, at the same four deviations. The body of `qefsSolveAsTheManagedOnesDo` became a helper that both use. Its job solves by the probabilistic solver when a flag is set, on QEFs made as SVD ones, which add normals as they are and keep the mass point of every sample. In step 2 the flag becomes the solver type.
   - **Measured:**
     - **Burst against managed:** vertices about 3.1e-8 / σ² apart at every deviation. That is 7.8e-5 at 0.02, 1.3e-5 at 0.05, 4.8e-7 at 0.25 and 3e-8 at 1. Errors are within 1.1e-6 of each other, relative, and mass points are the same to the bit. The tolerance for vertices is 1e-5 + 5e-8 / σ².
     - **One sample at 0.02:** its vertex is 3.3e-5 from the sample, not on it. That is rounding of `AᵀA p` against `Aᵀb`, divided by a pull of 0.0004. The degenerate cases are held to 1e-4, as the closed form is.
     - **Outside Unity,** in a dotnet console project around the same files, the closed form held to 4.6e-6 in all 24 cases, and the Burst struct gave the managed vertex to the bit.
   - **Deliberate mistakes:** 15, 11 in the managed solver and 4 in Burst, all caught (`probabilistic-quadrics-step1/mutations.txt` in the probes folder):
     - in the pull: one factor of the deviation, no count;
     - in the right side: no `AᵀA p`, a pull toward the origin;
     - in the LDLᵀ: four slips in the managed solve and one in Burst's;
     - in the error: taken at the mass point;
     - in `combine`: the mass point or the count left out.
   - **Suites:** edit mode 3823 of 3823 in 405 s, which is 42 more than step 0, and play mode 21 of 21. The 480 `Start` hashes are identical to step 0's.
2. **The solver in the CPU implementations (medium).** The enum value, the setting and its inspector, the managed `switch`es, `BurstSettings` and the five `create` calls. The 41 tests take it in. **Measured here,** on the project's models and shapes, against SVD:
   - distance of vertices from single shapes and from the edges and corners of cubes, upright and turned;
   - vertices outside their cells;
   - vertex counts of Adaptive and Manifold at the scenes' thresholds;
   - the suite's time.

   The default of 0.05 is confirmed or changed here, and the questions under "Asked after step 2" are put. **Done 2026-11-18:**
   - **Built:**
     - `QEFSolver.Type.Probabilistic`, after `SVD`.
     - `Voxelizer.normalDeviation`, labelled Normal Deviation, from 0.02 to 1 with a default of 0.05, and in the `SurfaceExtractor` interface beside the solver.
     - **The inspector** leaves out Normal Deviation under Simple and SVD, and Minimizer Iterations under Probabilistic (decision 2). Checked with a probe for Uniform and Occupancy under each solver, since no test draws an inspector. Voxelizers selected together fall back to Unity's generic inspector, as they did before.
     - **Managed:** the four `switch`es make a `ProbabilisticQEF` with the deviation.
     - **Burst:** `BurstQEF` keeps its deviation from `create`, which all five callers give it. Its `combine` keeps the mass point of every sample for this solver, as for Simple, and `solve` dispatches to it. `BurstSettings` carries the deviation for Uniform, Adaptive and Manifold, and Occupancy's job and `BurstOccupancyCells.solve` take it as well. Step 1's public `solveProbabilistic` became a private part of `solve`.
     - **GPU:** Uniform's driver takes the deviation, to fit the interface, and does nothing with it yet. The inspector still turns a GPU voxelizer's solver into SVD. Both change in step 3.
   - **Tests:**
     - **The 41 parametrized tests** took the new solver and passed without a change: the Burst-against-managed comparisons of all three algorithms, simplified or not, the topology tests and the tests of `placeVertex`. None needed a tolerance of its own (finding 13).
     - `BurstQEFTest`: the solver at its default in `qefsSolveAsTheManagedOnesDo`, which solves two of each group of QEFs before combining them, and at 0.02, 0.25 and 1 in `probabilisticQEFsSolveAsTheManagedOnesDo`. The flag of step 1 became the type.
     - `QEFSolverTest` makes the new solver, and `VoxelizerDefaultsTest` holds the default of 0.05.
     - **`NormalDeviationTest`,** new, 24 cases, all on a turned cube, with Adaptive and Manifold simplifying at 2e-4:
       - with the probabilistic solver, the contour changes between 0.05 and 0.5, in each of the seven CPU variants;
       - with Simple and SVD, it stays the same to the bit;
       - managed and Burst give the same contour at 0.5, for Uniform, Adaptive and Manifold. A QEF made with the default deviation in place of the setting changes nothing at the default, which is why this one is needed (see the mistakes).
     - **`RotatedCubeTest`** runs for SVD and the probabilistic solver, 96 cases. Under the new solver, Adaptive and Manifold test their corners and edges without simplification, at SVD's tolerance; with it they missed corners by up to 0.14 cells (under Measurements, Step 2, The rotated cube). That is asked below.
     - **The `Start` hash probe** hashes the new solver too, on the CPU: the 480 lines that were there are unchanged, and the new ones are the reference for the later steps.
   - **Deliberate mistakes:** 18, all caught (`probabilistic-quadrics-step2/mutations.txt` and `mutations_again.txt` in the probes folder):
     - the deviation dropped from the settings of each of Uniform, Adaptive, Manifold and Occupancy, or from `create`;
     - a fixed 0.05 in each place a QEF is made, managed and Burst;
     - the Burst solve falling through to SVD, managed Manifold making an SVD solver, another default.
     - **Two survived the first run:** managed Adaptive with a fixed deviation for its leaves only, or for its nodes only. The QEFs made in the other place still moved the contour, and at the default nothing changed at all. The managed-and-Burst test at 0.5 was added for them and catches both, and four more mistakes of the kind in Burst.
   - **Measured:** under Measurements, Step 2. The default of 0.05 stands:
     - On single shapes at 32³, 0.05 is the lowest deviation that leaves no more vertices outside their cells than SVD does, within a quarter: 2.8% against 2.6%. At 0.03 it is 4.1%, at 0.02 6.9%, on the curved shapes.
     - Corners and edges are off by a few thousandths of a cell more than with SVD. The most is 0.035 cells at the turned pyramid's tip, where four faces meet, against SVD's 0.017.
     - The worst vertex off a shape is nearer than with SVD: 0.10 cells against 0.18, at the cone.
     - **Where this solver was expected to win, these shapes give it no chance.** Their normals are exact, without noise, and at every edge of them the normals of the two faces are at least 60° apart, the hexagonal prism's being the closest. The shallow ridges of the feasibility study are not among them.
   - **Suites:** edit mode 4087 of 4087 in 432 s, play mode 21 of 21. That is 264 tests more than step 1 and 27 s longer. No test came near two minutes.
   - **`Start` hashes:** 704 lines, the 480 of step 0 identical, and 224 for the probabilistic solver: Uniform, Adaptive and Manifold in Managed and Burst, and Occupancy in Burst. They are the reference for steps 3 to 5, in `probabilistic-quadrics-step2/hashes_step2.txt` in the probes folder.
3. **GPU (medium).** `QEF.hlsl`, both shaders' configurations and drivers, the inspector's rule, the GPU tests. **Done 2026-11-18:**
   - **Built:**
     - `QEF.hlsl`: `minimizeProbabilistic`, the same LDLᵀ as the managed and Burst solves, credited. `solveQEF` takes the solver, the SVD sweeps and the deviation, and chooses between the two solves on the move from the mass point.
     - **Both shaders:** `qefSolver` and `normalDeviation` at the end of `Configuration`, on both sides, field for field.
     - **Drivers:** Uniform's driver passes them on. Occupancy's driver is given the solver and the deviation for the first time.
     - **Simple on the GPU:** `QEFSolver.shaderSolverOf`, in a new `GPU/QEFSolver.GPU.cs`, gives the shaders SVD for it. The inspector turns only Simple into SVD on the GPU now.
   - **Tests:**
     - **`ProbabilisticTest`,** new, in play mode beside `SVDTest`, 5 cases. A kernel solves the sums of a QEF, laid out as `SVDQEF.Data`, through `solveQEF`, for 200 sets of samples each: the probabilistic solver against `ProbabilisticQEF` at 0.02, 0.05, 0.25 and 1, and the SVD solver against `SVDQEF`.
       - **Measured:** the GPU's vertices are about 2e-8 / σ² from the managed ones, the most 4.9e-5 at 0.02, a little closer than Burst's. The SVD solver's are within 1.6e-7.
       - **Tolerances:** those of `BurstQEFTest`, 1e-5 + 5e-8 / σ².
     - **`UniformDualContouringGPUTest`:** the vertices match the CPU's for both solvers, on the test models and the turned shapes. New: `simpleSolverContoursAsSVD`.
     - **`OccupancyDualContouringGPUTest`:** the contour is Burst's for both solvers, on the test models and the turned shapes. New: `simpleSolverContoursAsSVD`.
     - **`NormalDeviationTest`** has Uniform and Occupancy on the GPU too, 30 cases.
     - **The GPU writes its vertices in the order its threads come to them,** which differs from one contour to the next. My first versions of the new tests compared two GPU contours index by index, and failed for that alone. They compare the vertices in order of position now, with `TopologyTest.inOrder`.
     - **The `Start` hash probe** hashes the new solver on the GPU too.
   - **Deliberate mistakes:**
     - **In C#, 6, all caught** (`probabilistic-quadrics-step3/mutations_cs.txt` in the probes folder): each driver giving SVD or a fixed deviation, the Occupancy component passing a fixed deviation, and `shaderSolverOf` always giving SVD. One I did not run: `shaderSolverOf` giving Simple its own number would change nothing, since the shaders take anything but the probabilistic solver as SVD.
     - **In the shaders, none:** skipped at the user's decision, "Skip the shader mistakes pass." The first of them, `if( false )` in place of the solver's branch in `solveQEF`, stopped the editor twice before the test run so much as started. Both times the guard took it for a hang and killed and relaunched the editor, at 20:35 and 20:47, while the user was working in it. Why that mistake stalls the editor was not looked into.
     - **What covers the shaders instead:** their solve is the arithmetic of the Burst solve, whose mistakes were all caught, and the GPU tests hold it to the managed solve and the contours to the CPU's.
   - **Suites:** edit mode 4105 of 4105 in 425 s, play mode 26 of 26.
   - **`Start` hashes:** 768 lines, in `probabilistic-quadrics-step3/hashes_step3.txt`. The 480 of step 0 and the 224 of step 2 are identical, and 64 are new, for the probabilistic solver on the GPU. Uniform and Occupancy on the GPU run twice each, and their two runs agree.
4. **Interactive editor and scenes (small).** `select`, the signature, the editor tests. The six scenes, with the user's go-ahead.
5. **Timings and documentation (small).** Burst and GPU timings of the three solvers at 64³ and 128³, with Unity in front. README: the solver in the implementations section, what it is good and bad at (findings 4, 6 and 7), the setting, the paper and the MIT credit. `AGENTS.md`: the paper among the algorithms.

## Measurements

### Baseline timings (step 0, 2026-11-18)

A whole contour of the three balls, read back from the GPU, without mesh simplification or remeshing, normals creased at 180°. Median of 7 runs after one to warm up, in ms, with Unity in front for every call. "Simplified" is Adaptive's simplification and Manifold's clustering at 2e-4, which solve a QEF for every node of the octree.

| Algorithm | Implementation | Contour | 64³ SVD | 64³ Simple | 128³ SVD | 128³ Simple |
|---|---|---|---|---|---|---|
| Uniform | Burst | plain | 8.0 | 7.9 | 39.2 | 43.1 |
| Occupancy | Burst | plain | 15.0 | 14.7 | 64.3 | 65.6 |
| Adaptive | Burst | plain | 46.6 | | 348.4 | |
| Adaptive | Burst | simplified | 6.9 | 7.0 | 35.0 | 34.2 |
| Manifold | Burst | plain | 15.0 | | 66.9 | |
| Manifold | Burst | simplified | 8.0 | 8.0 | 36.8 | 35.4 |
| Uniform | GPU | plain | 21.1 | | 124.9 | |
| Occupancy | GPU | plain | 7.1 | | 24.4 | |

Uniform Burst with Simple at 128³ had one slow run of 64 ms among its seven; its fastest was 37.2 ms.

### Step 2 (2026-11-18)

From the Burst implementations, which give the managed meshes within a few thousandths of a cell. Probes `PQMeasureProbe.cs` and `PQCubeProbe.cs`, with their rows in `probabilistic-quadrics-step2` in the probes folder. Distances are in cells.

#### Single shapes

The nine shapes of `ShapeContourTest` and a box and a ball, upright and turned, at 32³, Uniform Dual Contouring. Means are over the shapes, worst cases over all of them.

| Solver | Off the shape, mean | worst | Corners from the nearest vertex, mean | Edges from the mesh, mean | worst | Vertices outside their cells |
|---|---|---|---|---|---|---|
| SVD | 0.0135 | 0.184 | 0.013 | 0.0045 | 0.017 | 2.56% |
| 0.02 | 0.0135 | 0.079 | 0.013 | 0.0047 | 0.019 | 6.88% |
| 0.03 | 0.0135 | 0.083 | 0.014 | 0.0049 | 0.022 | 4.10% |
| 0.05 | 0.0133 | 0.104 | 0.018 | 0.0058 | 0.035 | 2.78% |
| 0.07 | 0.0132 | 0.121 | 0.026 | 0.0077 | 0.056 | 2.54% |
| 0.1 | 0.0130 | 0.131 | 0.045 | 0.0127 | 0.105 | 2.33% |

- **The worst corner** is 0.26 cells for every solver: a corner of the turned box in a cell whose corners show two pieces of surface, which gets one vertex. Manifold Dual Contouring gives such a cell a vertex for each piece, and its worst corner is 0.021 with SVD and 0.035 at 0.05.
- **Adaptive Dual Contouring** without simplification gives Uniform's numbers to the digit.
- **Vertices outside their cells** at 0.02 and 0.03 are on the curved shapes. On the torus at 0.03 they are 78 of 1124 upright and 108 of 1198 turned, against SVD's 16 and 13. At 0.05 they are 31 and 34.
- **Occupancy-Based Dual Contouring** hardly changes with the solver. Its corners are 0.28 cells from the nearest vertex on average and its edges up to 0.79 from the mesh, with SVD and at every deviation: its searches decide them, not the solve (finding 13 of its plan). It keeps every vertex in its cell.

#### Vertex counts

Adaptive and Manifold Dual Contouring at 32³ leaves, at four error thresholds. The test models, the turned shapes and new shapes of `TopologyTest`, a cube turned about a diagonal, and a cube of 0.6 at the origin. Manifold at 2e-4, the threshold of the scenes:

| Model | SVD | Simple | 0.02 | 0.05 | 0.1 |
|---|---|---|---|---|---|
| Sphere | 415 | 415 | 415 | 415 | 415 |
| Three balls | 852 | 936 | 852 | 852 | 884 |
| Cubes and sphere | 440 | 688 | 440 | 440 | 488 |
| Sphere cutaway | 872 | 1124 | 872 | 872 | 872 |
| Turned shapes | 547 | 1048 | 542 | 637 | 701 |
| New shapes | 1368 | 1767 | 1334 | 1385 | 1501 |
| Turned cube | 104 | 579 | 124 | 251 | 325 |
| Cube of 0.6 | 8 | 56 | 8 | 56 | 56 |

- **Curved models** simplify as far as with SVD, and a little further at the smallest thresholds: the sphere keeps 1534 vertices at 1e-5 against SVD's 1578.
- **Models with features** keep more, as finding 7 has it: the turned cube 2.4 times as many at 0.05.
- **Adaptive** is alike: the turned cube keeps 497 against SVD's 453 at 2e-4, and the cube of 0.6 keeps 56 against 8.
- **The simple solver** keeps the most in every model with features.

#### The rotated cube

`RotatedCubeTest`'s cube at 16³, simplified at 2e-4 as there. How far the farthest corner is from the nearest vertex:

| Algorithm | Turn | SVD | 0.02 | 0.03 | 0.05 | 0.1 |
|---|---|---|---|---|---|---|
| Uniform | about an axis | 0.013 | 0.014 | 0.015 | 0.019 | 0.038 |
| Uniform | about a diagonal | 0.012 | 0.013 | 0.014 | 0.018 | 0.045 |
| Adaptive and Manifold | about an axis | 0.014 | 0.050 | 0.035 | 0.075 | 0.179 |
| Adaptive and Manifold | about a diagonal | 0.012 | 0.032 | 0.053 | 0.136 | 0.071 |

The edges are as far as the corners, but for 0.1 about a diagonal: 0.099 cells in Adaptive and 0.089 in Manifold. Under simplification nothing but the error threshold holds a cluster's vertex to the corner, which is why the numbers do not grow evenly with the deviation.

### Feasibility

All from the feasibility experiment of 2026-11-18, not from the Unity pipeline. It is a console program that compiles `BurstQEF.cs` unmodified and samples exact shapes as the project does: a 64³ grid one unit wide, six bisection steps, normals by central differences of 1e-4, all in floats. It is kept outside the repo, in the session's probes folder under `probabilistic-quadrics`. Distances are in cells.

#### Single cells, clean data

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

#### Single cells, normals off by about 3°

Gaussian noise of 0.05 on each component of every normal. Share of vertices outside their cells, and how far the worst one is.

| Case | SVD | `σ` 0.01 | 0.03 | 0.05 | 0.07 | 0.1 |
|---|---|---|---|---|---|---|
| Ball | 0.2%, 0.02 | 53%, 1.6 | 22%, 0.46 | 7.8%, 0.23 | 3.6%, 0.14 | 1.5%, 0.05 |
| Turned box | 2.2%, 0.45 | 60%, 2.0 | 24%, 0.82 | 9.9%, 0.50 | 5.8%, 0.47 | 3.4%, 0.45 |

The mean distance from the ball's surface is 0.014 for SVD and 0.013 from 0.03 upward.

#### Summed QEFs

Blocks of cells of a box turned 20°, 30° and 10° about the axes, each summed into one QEF. Share of blocks whose classical error at the vertex is under 1e-5, and the mean distance from the vertex to the block's edge or corner.

| Block | SVD | `σ` 0.02 | 0.05 | 0.1 |
|---|---|---|---|---|
| 4 cells | 100%, 0.002 | 100%, 0.008 | 98%, 0.041 | 88%, 0.13 |
| 8 cells | 98%, 0.003 | 94%, 0.026 | 69%, 0.13 | 57%, 0.37 |
| 16 cells | 95%, 0.0015 | 54%, 0.29 | 32%, 0.84 | 30%, 1.5 |

For a box along the axes every block is under 1e-5 up to `σ` = 0.05, and vertices are within 0.02 cells of their feature.

#### A moving shape

Each shape moved by a hundredth of a cell 150 times. How far a cell's vertex moves per step, on average.

| Shape | SVD | `σ` 0.03 | 0.05 | 0.1 |
|---|---|---|---|---|
| Ball | 0.011 | 0.041 | 0.023 | 0.013 |
| Turned box | 0.013 | 0.013 | 0.013 | 0.013 |

Steps over half a cell on the turned box: 780 for SVD and 649 at 0.05, of 1.3 million.

#### The solve

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

Answered 2026-11-18. Decisions 2 to 6 as recommended.

1. **Name and range of the setting.** Recommended: **Normal Deviation**, 0.02 to 1, default 0.05 (findings 5 and 8). The paper calls it `σ_n` and gives it in percent of the normal's length; 0.05 is its 5%.
   - **Answer:** "The setting should be called Probabilistic. You can name anything else." The range and default stand.
   - **Asked after step 1,** since it could be read two ways, and answered on 2026-11-18: "the setting" is the solver's value in QEF Solver, Probabilistic, and the deviation's name was left to me. It is Normal Deviation.
2. **What the inspector shows.** Recommended: Normal Deviation only under Probabilistic, and Minimizer Iterations hidden then, since neither means anything to the other solvers. The other way shows both always, which is simpler and leaves a dead setting in view. **Answer:** as recommended.
3. **The error for simplification and clustering.** Recommended: the classical QEF at the placed vertex (finding 11). **Answer:** as recommended.
4. **Port the authors' solve?** Recommended: yes, the 25 lines of `minimizer()`, credited in a comment and in the README. The licence is MIT, as this project's is. **Answer:** as recommended. Ported in step 1.
5. **Example scenes.** Recommended: six `Probabilistic Solve` scenes beside the `SVD Solve` ones. The other way is none, since the interactive editor shows the solver on any model. **Answer:** as recommended.
6. **Editor and README in this plan?** Recommended: yes, as steps 4 and 5. **Answer:** as recommended.

### Asked after step 2

Put to the user on 2026-11-18, and answered the same day: "1. Keep as is 2. Not needed Continue with step 3." The rotated cube's corners and edges stay tested without simplification for the new solver, and the inspector gets no warning.

- **The default,** if the project's models disagree with 0.05. They do not (step 2, Measured), so it stays, and nothing is asked.
- **Tests that do not hold for this solver as written** (finding 13): which get a tolerance of their own and which leave the solver out.
  - **Only one did not hold:** the corners and edges of `RotatedCubeTest` under simplification, which the plan's tests had added and the 41 parametrized tests are not among. They missed by up to 0.14 cells against 0.002.
  - **Recommended, and how it is now:** test the new solver's corners and edges there without simplification, at SVD's tolerance, and keep its simplified contours in the closed-surface and on-the-cube tests. Under simplification only the error threshold holds a vertex to a corner, so no tolerance would follow from anything but the one model it was measured on.
  - **The other way:** a tolerance of its own under simplification, a quarter of a cell, with the measured 0.14 behind it.
- **Whether Adaptive and Manifold should warn** in the inspector that this solver simplifies less than SVD. Recommended: no warning in the inspector, which has none for anything else; the README says it in step 5, with the counts above.
