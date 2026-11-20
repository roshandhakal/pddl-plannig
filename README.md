# 2D Task and Motion Planning with PDDLStream

A research prototype for **task and motion planning (TAMP)** in a simplified 2D block-world environment. It combines symbolic action descriptions with geometric feasibility checks and motion planning through [PDDLStream](https://github.com/caelan/pddlstream).

> **Status:** Legacy research experiment. The Docker configuration and upstream dependencies have not been verified against current environments.

## What the project includes

- `plan/domain.pddl` and `plan/stream.pddl`: symbolic actions and streams for the planning problem.
- `plan/environments/blockworld/`: 2D environment definitions and geometric primitives.
- `plan/scripts/run.py`: builds a PDDLStream problem, invokes a planner, and can visualize a solution.
- `Dockerfile` and `Makefile`: an older container-based development and execution workflow.

The example explores how actions such as **move, pick, and place** can be planned together with constraints on poses, collisions, and motion.

## Running the example

The repository's original workflow is:

```bash
make build
make plan
```

`make plan` already depends on the `build` target, so running it by itself also builds the image.

**Before running:** review the [Makefile](Makefile) and [Dockerfile](Dockerfile). The legacy image uses an older CUDA/Ubuntu stack and VirtualGL. The Makefile executes `xhost +`, which broadly relaxes X-server access controls; do not run that command uncritically on a shared or untrusted system. The old dependencies may need maintenance to build today.

The Python entry point includes options for choosing example problems, controlling collision checks, setting a random seed, and visualizing planning output. See `plan/scripts/run.py` for the options supported by this snapshot.

## Relationship to upstream code

The core experiment is based on the [PDDLStream continuous TAMP example](https://github.com/caelan/pddlstream/tree/main/examples/continuous_tamp). The entry-point source explicitly notes that substantial portions came from that example, with modifications for this 2D environment. See upstream PDDLStream for the underlying planning framework.

## Context

This repository is an exploration of **integrating symbolic task choices with continuous geometric constraints**, not a deployed robot control stack or a claim of a novel standalone planning framework.

For a newer symbolic planning pipeline with documentation and tests, see [BEHAVIOR-1K Task Planning](https://github.com/roshandhakal/behavior1k-taskplanning).
