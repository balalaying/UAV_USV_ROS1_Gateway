# Project Refactor Phase 1 Report

## Objective

Phase 1 establishes a clear main entry, launch classification, documentation
destinations, package ownership index, script classification and a frozen Topic
and TF reference. It is deliberately a metadata/documentation phase.

## Files Modified

| File | Change |
|---|---|
| `README.md` | Declares the frozen complete 332 main entry and all 17 package classifications. |
| `src/uav_usv_bringup/README.md` | Declares the main launch and its duplicate-start safety rule. |
| `src/uav_usv_bringup/launch/fleet_dynamic_capture_live_perception.launch.py` | Adds a docstring-only `REFRACTOR_METADATA` header. |
| `PROJECT_STRUCTURE.md` | Records Phase 1 status and document-directory state. |
| `PROJECT_REFACTOR_REPORT.md` | Records Phase 1 additions and nonfunctional scope. |

## Files Added

- `docs/README.md`
- `docs/LAUNCH_INDEX.md`
- `docs/SCRIPT_CLASSIFICATION.md`
- `docs/LEGACY_INTERFACE_INVENTORY.md`
- This report
- README placeholders in `docs/architecture`, `simulation`, `perception`,
  `world_model`, `base_station`, `communication`, `deployment`, `validation`,
  and `archive`

## Main and Compatibility Entrances

- **Frozen primary:** `uav_usv_bringup/fleet_dynamic_capture_live_perception.launch.py`
- **World-only inspection:** `uav_usv_bringup/simulation_332_scenario.launch.py`
- **Compatibility:** all `uav_usv_sim` launch files remain supported.
- **Historical/validation:** defense, lighthouse, buoy, calibration, old Qt and
  earlier capture launches remain present and are classified in
  [LAUNCH_INDEX.md](LAUNCH_INDEX.md).

No launch was removed. No launch path, argument, package name, topic or frame
was changed.

## Documentation Movement

Moved Markdown files: **none**.

The required future directories now exist but contain only README placeholders.
`docs/README.md` is the mapping table. This intentionally avoids breaking
existing repository links before a dedicated link-safe documentation migration.

## Package Decision

All 17 packages are retained. No package is approved for deletion. Package
roles are indexed in the root README and [PROJECT_STRUCTURE.md](../PROJECT_STRUCTURE.md).

## Topic and TF Freeze

[TOPIC_ARCHITECTURE.md](TOPIC_ARCHITECTURE.md) remains the authority. The legacy
inventory records `/maritime/*` and `landing_boat/*` references without applying
any migration. The 332 map-frame architecture remains unchanged.

## Verification Plan and Results

| Check | Result | Notes |
|---|---|---|
| Python launch syntax | Pass | All 35 `*.launch.py` files compiled with `python3 -m py_compile`. |
| Workspace build | Pass | `colcon build --symlink-install`: 17/17 packages completed. |
| Primary 332 launch smoke | Pass, control-safe mode | Started the frozen entry with PX4, DDS, RViz, Qt, LV-DOT and camera-LiDAR fusion disabled to avoid duplicate control/GUI during smoke validation. Gazebo, 332 entities, three USV Nav2 stacks, sensor bridges, preprocessing, fusion, World Model and Base Station Service started. |
| 3 UAV / 3 USV presence | Pass | Fleet pose TF publisher reported `uav_01..03`, `usv_01..03`, `friendly_ship`, and `enemy_ship`; all three UAV camera topics and three USV camera topics appeared. |
| Three Mid-360 paths | Pass | `fleet_usv_01..03_mid360_pointcloud_bridge` nodes and three `/fleet/uplink/usv_0x/mid360/points` topics appeared; first clouds were received. |
| LV-DOT node | Pass | Standalone native ROS 2 lifecycle launch configured and activated for `usv_01`, then cleanly exited. |
| `/fleet/perception/fused_targets` | Pass | Present during primary smoke launch. |
| `/fleet/world_model` | Pass | Measured approximately 4.99-5.00 Hz. |
| `/base_station/state` | Pass | Measured approximately 4.99-5.00 Hz. |
| Qt client | Pass | `dynamic_capture_console.launch.py` started `fleet_base_station_gui`, observed live USV camera/lidar/navigation uplink, and exited cleanly on SIGINT. |
| Gateway Demo | Pass | `remote_summary_gateway.launch.py` started its read-only WebSocket gateway on a temporary port and exited cleanly on SIGINT. |

### Known Validation Limit

PX4/DDS Offboard flight, camera-LiDAR fusion and LV-DOT processing were not run
together in this Phase 1 smoke test. This was intentional: Phase 1 does not
alter control or perception behavior, and the objective is to validate launch
and documentation consolidation without triggering a flight mission. Their
previous validated flows remain unchanged; a future full-regression phase must
run the normal primary command with PX4 enabled.

## Next Phase Allowed Work

After explicit approval, Phase 2 may add metadata headers to the remaining
launch files, relocate Markdown with compatibility notices, then introduce
launch-directory wrappers one domain at a time. It must retain original launch
names and run the 332 regression suite after each move.
