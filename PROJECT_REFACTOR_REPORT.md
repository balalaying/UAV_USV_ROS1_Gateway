# Project Refactor Report

## Scope and Safety Boundary

This began as a planning-only audit for the UAV_USV repository. Project Refactor
Phase 1 adds documentation indexes, empty documentation destination directories,
and nonfunctional main-entry metadata. No package, launch implementation,
configuration, source algorithm, topic, TF edge, or model was moved, renamed,
deleted, or functionally changed.

Protected systems include LV-DOT, camera-LiDAR fusion, perception fusion, Fleet
World Model, Base Station Service, PX4 control, capture management,
FleetCommand, TF, and ROS topic interfaces.

## Inventory Result

The workspace contains 17 ROS 2 packages. The active architecture is coherent:
interfaces -> vehicle/sensor layers -> perception -> Fleet World Model -> Base
Station Service -> Qt/gateway clients. The primary consolidation issue is not
duplicate runtime algorithms; it is historical demonstration and validation
launches living beside the current 332 integrated workflow.

## Findings

### 1. Package classification

- **Official runtime packages:** `uav_usv_interfaces`, `uav_usv_gazebo`,
  `uav_usv_bringup`, `uav_usv_uav_control`, `uav_usv_usv_control`,
  `uav_usv_lv_dot_core`, `uav_usv_lv_dot`, `uav_usv_perception`,
  `uav_usv_mission`, `uav_usv_base_station`, and `uav_usv_fleet_gateway`.
- **Official compatibility package:** `uav_usv_sim`. It still owns working
  early simulation, PX4/Nav2 and teaching flows and must not be removed.
- **Planned ownership packages:** `uav_usv_description`, `uav_usv_navigation`,
  `uav_usv_localization`, `uav_usv_colregs`, and `uav_usv_tests`. Their
  READMEs describe future ownership; they are not duplicate implementations.

Recommendation: retain every package in this phase. Add a small package index
to the root README during a later approved documentation pass.

### 2. Launch duplication

The Gazebo/PX4/perception stack appears in multiple historical combinations:
single UAV/USV, dual UAV capture, defense, lighthouse, buoy, calibration,
LV-DOT validation and full 332 demonstrations. Starting more than one such
launch can create duplicate Gazebo worlds, bridges, PX4 instances, Fusion nodes
or Qt clients.

Recommendation: declare the following as the only primary full-system entry:

```bash
ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py
```

Keep all alternatives runnable but later label them `validation`,
`compatibility`, or `deprecated` in their module comments and docs. Do not
delete their files.

### 3. Configuration duplication

Configuration currently lives in the package that consumes it: Gazebo GUI/RViz,
perception parameters, gateway settings, Base Station state, and legacy Nav2
settings. This is operationally valid but makes discovery difficult.

Recommendation: use the taxonomy in `PROJECT_STRUCTURE.md` for future moves.
Before merging YAML files, compare every launch argument and parameter value;
preserve package-relative installation paths and old parameter names.

### 4. Source and tool overlap

There are valid overlapping roles rather than automatically removable code:

- `uav_usv_mission` contains mission-time Gazebo bridges, task code, the world
  model, behavior-shadow work and Qt client code.
- `uav_usv_perception` contains runtime adapters plus calibration, evaluation
  and visualization helpers.
- `uav_usv_sim` and `uav_usv_gazebo` both contain models/world assets because
  the former is compatibility-oriented while the latter owns the 332 world.
- Gateway, Qt and Base Station Service are distinct presentation/transport
  clients and should not be merged without an API migration plan.

Recommendation: classify scripts as `runtime`, `validation`, `tool`, or
`compatibility` first. Only then move non-runtime scripts to `archive/` with an
installed compatibility wrapper. No source move is approved by this audit.

### 5. Topic and TF assessment

`docs/TOPIC_ARCHITECTURE.md` records the intended interfaces. The canonical
fleet output is `/fleet/world_model`; `/base_station/state` supplies the
read-only base-station reference for clients. Raw sensor frames stay vehicle
local, while fleet targets are map-frame products. Existing legacy topic
documents need later review, but no topic rename is recommended now.

### 6. Documentation overlap

The root `docs/` directory contains architecture reports, validation reports,
sensor contracts, gateway reports and historical notes in one flat list.
Several documents cover related layers at different dates. This is an archive
quality issue, not a reason to delete evidence.

Recommendation: later move documents to the taxonomy below, retain original
filenames, and add an `archive/README.md` that states whether a document is
current, superseded or historical.

```text
docs/{architecture,simulation,perception,world_model,base_station,
      communication,deployment,validation,archive}/
```

## Proposed Future Consolidation

| Area | Future action | Compatibility protection |
|---|---|---|
| Launches | Group by `simulation`, `perception`, `base_station`, `deployment`, `validation`, `deprecated` | Keep original launch filenames as wrappers. |
| Config | Create domain folders under the owning package first | Preserve all launch arguments and old YAML paths. |
| Scripts | Label runtime versus validation/tool sources | Keep executable name and install rule stable. |
| Models | Declare `uav_usv_gazebo` 332 assets canonical; retain `uav_usv_sim` legacy assets | Do not merge model paths before world regression tests. |
| Docs | Rehome by domain and add indexes | Keep links or redirect stubs at legacy paths. |
| Topics/TF | Document aliases and owners | No rename or frame change during cleanup. |

## Recommended Execution Plan After Approval

1. Create indexes and deprecation headers only; perform no physical moves.
2. Consolidate documentation with link validation.
3. Introduce new launch subdirectories and compatibility wrapper launches.
4. Consolidate one configuration domain at a time, followed by parameter dump
   comparison.
5. Archive validation/demo tools only after executable smoke tests prove their
   replacements cover the use case.
6. Run integrated regressions after every change: 332 spawn, UAV PX4 agents,
   USV agents, Mid-360, cameras, LV-DOT, perception fusion, Fleet World Model,
   Base Station Service, Qt and gateway.

## Risk Analysis

- **High:** simultaneous legacy/full launches may duplicate PX4, Gazebo,
  bridges or perception producers. Mitigation: document one primary launch and
  label alternatives before reorganizing.
- **High:** moving model assets or configs can silently break package install
  paths, Gazebo resource resolution and world references. Mitigation: retain
  paths via wrappers/symlinks only after isolated world smoke tests.
- **Medium:** historical `/maritime/*` documentation may conflict with current
  `/fleet/*` interfaces. Mitigation: inventory aliases before publishing a
  migration guide.
- **Medium:** Qt, gateway and Base Station Service are currently evolving in
  the working tree. Mitigation: do not refactor presentation code until the
  dirty changes are committed or explicitly shelved by their owner.
- **Low:** planned packages have little implementation today. Mitigation: keep
  them as ownership boundaries rather than treating them as obsolete.

## Files Added by This Planning Phase

- `PROJECT_STRUCTURE.md`
- `PROJECT_REFACTOR_REPORT.md`
- `docs/TOPIC_ARCHITECTURE.md`

## Phase 1 Additions

- `docs/README.md` and destination-directory README placeholders;
- `docs/LAUNCH_INDEX.md`;
- `docs/SCRIPT_CLASSIFICATION.md`;
- `docs/LEGACY_INTERFACE_INVENTORY.md`;
- `docs/REFACTOR_PHASE1_REPORT.md`.

The root README, bringup README, and primary launch received only the frozen
main-entry metadata. No legacy launch was deleted or moved.

Phase 1 verification: all 35 launch modules passed Python syntax compilation;
all 17 ROS 2 packages built successfully; the 332 smoke launch started the
three Mid-360 bridges, three camera paths, fleet fusion, Fleet World Model and
Base Station Service. World Model and Base Station state each measured about
5 Hz. Qt, Gateway and the native LV-DOT lifecycle node were smoke-tested
separately. PX4/Offboard was intentionally disabled during this documentation
phase smoke test.
