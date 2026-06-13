# AAGV Robot

Agriculture AGV — an autonomous ground vehicle integrating a **Reeman AGV** base with an
**OpenManipulator-X** robotic arm for agricultural manipulation tasks.

## Stack

| Layer | Component |
|---|---|
| OS | Ubuntu 24.04 LTS |
| Middleware | ROS 2 Jazzy Jalisco |
| Motion planning | MoveIt 2 |
| Arm | OpenManipulator-X (ROBOTIS) |
| Base | Reeman AGV |
| Actuation | Dynamixel (Protocol 2.0) via U2D2 |
| Workspace | `~/aagv_ws` |

## Repository layout

```
.
├── README.md
├── .gitignore
└── docs/
    └── SETUP_LOG.md      # setup + progress log (reproducible steps, TODO)
```

> This repo is the project's **backup and progress record**. Setup steps, decisions,
> and open tasks live in [`docs/SETUP_LOG.md`](docs/SETUP_LOG.md). The commit history
> doubles as a progress timeline.

## Package strategy — hybrid (apt + source)

- **apt** for released, unmodified dependencies (MoveIt 2, ros2_control, controllers,
  dynamixel-sdk).
- **source** (in a ROS workspace `src/`) only for what isn't released to the Jazzy apt
  repo or is developed in-house:
  - OpenManipulator-X — `ROBOTIS-GIT/open_manipulator`, branch `jazzy`
  - Reeman AGV driver — vendor source/SDK
  - custom packages (e.g. `aagv_bringup`)

See `docs/SETUP_LOG.md` for the exact commands.

## Status

Early setup. Environment and workspace are up; robot packages and hardware bring-up
are in progress. See the TODO section in the setup log.
