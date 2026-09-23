# UAV_USV Project Structure

> Status: Project Refactor Phase 1 baseline, 2026-08-01. Phase 1 adds only
> indexes, documentation directories and metadata. It does not move, delete,
> rename, or change any runtime interface.

## Current Canonical Runtime

The current integrated 3 UAV + 3 USV scenario uses:

- World: `src/uav_usv_gazebo/worlds/heterogeneous_332.sdf`
- Main integrated launch: `uav_usv_bringup/fleet_dynamic_capture_live_perception.launch.py`
- Fleet state contract: `/fleet/world_model`
- Base-station state: `/base_station/state`
- Control contract: `FleetCommand`, `VehicleState`, `CommandAck`, and `ControlLease`
- Fleet reference frame: `map`

The source-mux default remains `ground_truth`. LV-DOT, camera-LiDAR fusion, and
fleet fusion are Shadow-mode perception sources. This is an operational choice,
not a refactor target.

The primary entry is now explicitly frozen in the root README, the bringup
README, and the launch file itself. See [Launch index](docs/LAUNCH_INDEX.md).

## Package Responsibilities

| Package | Classification | Responsibility | Refactor disposition |
|---|---|---|---|
| `uav_usv_interfaces` | official | Shared ROS messages and service contracts | Keep as the contract boundary. |
| `uav_usv_gazebo` | official | Canonical Gazebo worlds, environmental assets, fleet models, simulation tools | Keep; make `heterogeneous_332.sdf` the documented primary world. |
| `uav_usv_bringup` | official | Integrated system launch composition, RViz and top-level configuration | Keep; later split launch files into domain subdirectories without changing launch names. |
| `uav_usv_uav_control` | official | PX4 / DDS / offboard UAV agents | Keep unchanged. |
| `uav_usv_usv_control` | official | USV motion and safety control agents | Keep unchanged. |
| `uav_usv_lv_dot_core` | official | ROS-independent native LV-DOT algorithm layer | Keep unchanged. |
| `uav_usv_lv_dot` | official | Lifecycle ROS 2 LV-DOT wrapper and diagnostics | Keep unchanged. |
| `uav_usv_perception` | official | Sensor adapters, preprocessing, camera-LiDAR association, fusion, visualization helpers | Keep; distinguish runtime nodes from validation tools in documentation. |
| `uav_usv_mission` | official | Capture, task, Gazebo bridge, world-model and behavior-shadow nodes, Qt client | Keep; no behavior changes in cleanup. |
| `uav_usv_base_station` | official | Read-only base-station service and base-station reference state | Keep as the client data boundary. |
| `uav_usv_fleet_gateway` | official | Read-only ROS 2 to WebSocket/remote gateway and web demos | Keep; align its docs with the Base Station Service boundary in a later compatibility phase. |
| `uav_usv_sim` | official-compatibility | Original single/early UAV-USV world, Nav2, PX4 and educational demos | Keep intact; label legacy/compatibility entry points rather than deleting them. |
| `uav_usv_description` | planned | Future canonical robot descriptions, meshes and sensor extrinsics | Keep as a migration destination; active models still live in simulation packages. |
| `uav_usv_navigation` | planned | Future Nav2/MPPI and maritime navigation ownership | Keep; active stack remains under `uav_usv_sim`. |
| `uav_usv_localization` | planned | Future SLAM, GNSS/RTK and production TF ownership | Keep; Gazebo ground-truth TF remains current simulation fallback. |
| `uav_usv_colregs` | planned | Future COLREGs risk and decision layer | Keep; do not confuse it with old scenario demos. |
| `uav_usv_tests` | planned | Cross-package smoke, bag and contract tests | Keep; populate gradually after interfaces are frozen. |

No package is recommended for deletion in this first phase.

## Launch Classification

### Primary launches

| Launch | Role |
|---|---|
| `uav_usv_bringup/fleet_dynamic_capture_live_perception.launch.py` | Full 332 integrated simulation, perception, world model, Base Station Service and operator clients. |
| `uav_usv_bringup/fleet_dynamic_capture.launch.py` | Fleet capture runtime composition without the live-perception presentation set. |
| `uav_usv_bringup/simulation_332_scenario.launch.py` | 332 Gazebo scenario only; use for environment and model inspection. |
| `uav_usv_base_station/base_station_service.launch.py` | Standalone read-only Base Station Service. |
| `uav_usv_lv_dot/lv_dot.launch.py` | Native LV-DOT lifecycle wrapper. |
| `uav_usv_perception/perception_layer.launch.py` | Standard perception adapters, fusion and source mux. |
| `uav_usv_perception/camera_lidar_fusion.launch.py` | Camera-LiDAR fusion composition. |
| `uav_usv_fleet_gateway/remote_sensor_dashboard.launch.py` | Web/gateway sensor dashboard demonstration. |

### Validation and calibration launches

Keep these runnable, but mark them as validation-only in a later documentation
pass: `mid360_sensor_demo`, `minimal_dynamic_capture`, `dual_uav_dynamic_capture`,
`camera_lidar_calibration_debug`, `lv_dot_shadow`, `lv_dot_tuning`,
`lv_dot_fusion_validation`, and `multisensor_fusion_validation`.

### Historical or compatibility launches

The following remain valuable for teaching, regression and compatibility but are
not canonical 332 production entrances: `All_Qt`, `Qt_cooperation`,
`cooperative_capture`, `cooperative_response`, `defense`, `defense_sim`,
`unified_tasks_qt`, plus launches inside `uav_usv_sim` such as lighthouse,
keyboard, buoy, old PX4/Nav2 and COLREG scenario launches.

The proposed future layout is only a target; existing paths and names remain
unchanged until a dedicated compatibility-tested migration:

```text
launch/
  simulation/       # world-only and spawn-only entries
  perception/       # sensors, LV-DOT, fusion, calibration
  base_station/     # Qt, service and gateway clients
  deployment/       # full integrated field/demo start points
  validation/       # bags, diagnostics and calibration
  deprecated/       # historical launch wrappers, preserving old names
```

## Proposed Configuration Taxonomy

Configuration files should eventually be grouped by ownership, while preserving
their current installed paths and parameter names during migration:

```text
config/
  simulation/       # world, spawn, GUI, RViz and scenario settings
  vehicle/          # UAV/USV identity and vehicle-local parameters
  sensor/           # MID-360, RGB and RGB-D settings
  perception/       # preprocessing, LV-DOT, fusion and source-mux settings
  base_station/     # base_station.yaml and client display settings
  communication/    # gateway and WebSocket deployment settings
  navigation/       # Nav2/MPPI and local navigation parameters
```

## Source Layout Target

Runtime source should be documented as one of `nodes`, `adapters`, `fusion`,
`visualization`, `tools`, or `tests`. Existing source files remain where they
are in this phase. Future moves must retain installed executable names and add
compatibility wrappers before changing a path.

## Documentation Taxonomy Target

`docs/` is currently a flat historical record. Future organization:

```text
docs/
  architecture/     # global contracts and ownership
  simulation/       # worlds, models, Gazebo and PX4 validation
  perception/       # sensors, LV-DOT, fusion and calibration
  world_model/      # fleet world model and target contracts
  base_station/     # base station, Qt situation and display conventions
  communication/    # gateway, WebSocket and remote deployment
  deployment/       # installation and launch procedures
  validation/       # bag, smoke and regression reports
  archive/          # superseded reports retained for traceability
```

No document has been moved yet.

Phase 1 created these empty destination directories with README placeholders:
`architecture`, `simulation`, `perception`, `world_model`, `base_station`,
`communication`, `deployment`, `validation`, and `archive`. The mapping is in
[docs/README.md](docs/README.md), so no existing Markdown link needs changing.

## Safe Refactor Sequence

1. Freeze this inventory and identify the primary launch for each user workflow.
2. Add deprecation headers and successor links to historical launch/doc files.
3. Consolidate duplicate configuration only after a parameter-equivalence test.
4. Move files one domain at a time with compatibility wrappers and install-rule
   checks.
5. Run 332, sensor, LV-DOT, fusion, world-model, Base Station Service, Qt and
   gateway regression checks after every domain move.
