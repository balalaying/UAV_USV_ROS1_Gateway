# Documentation Index

This index was created during Project Refactor Phase 1. No existing Markdown
file has been moved yet, so all existing links and filenames remain valid.

## Current Authoritative Documents

- [Project structure](../PROJECT_STRUCTURE.md)
- [Project refactor report](../PROJECT_REFACTOR_REPORT.md)
- [Launch index](LAUNCH_INDEX.md)
- [Topic architecture](TOPIC_ARCHITECTURE.md)
- [Script classification](SCRIPT_CLASSIFICATION.md)
- [Legacy interface inventory](LEGACY_INTERFACE_INVENTORY.md)
- [Phase 1 report](REFACTOR_PHASE1_REPORT.md)

## Planned Documentation Mapping

| Future directory | Current documents to migrate later | Status |
|---|---|---|
| `architecture/` | `ARCHITECTURE_AUDIT_2026-07-13.md`, `CAPTURE_INTERFACE_FREEZE_V1.md`, `SCALABLE_CAPTURE_FRAMEWORK_2026-07-13.md` | Mapping only. |
| `simulation/` | `SIMULATION_332_SCENARIO_REPORT.md`, `SENSOR_INTERFACE.md`, `USV_SCAN_AND_GLOBAL_TF_VALIDATION.md` | Mapping only. |
| `perception/` | `LV_DOT_*.md`, `CAMERA_LIDAR_CALIBRATION_DEBUG_REPORT.md`, `MULTISENSOR_VALIDATION_REPORT.md`, `VISION_GUIDED_*.md`, `UAV_CAMERA_INTERFACE.md` | Mapping only. |
| `world_model/` | `FLEET_WORLD_MODEL_ARCHITECTURE.md`, `RADAR_FUSION_AUDIT_REPORT.md` | Mapping only. |
| `base_station/` | `BASE_STATION_*.md`, `QT_*.md` | Mapping only. |
| `communication/` | `FLEET_WEB_PROTOCOL.md`, `GATEWAY_DEPLOYMENT_INTERFACE.md`, `MOBILE_FLEET_DEMO_REPORT.md`, `REMOTE_*.md` | Mapping only. |
| `deployment/` | environment and reproduction guides | Mapping only. |
| `validation/` | capture, sensor, flight and shadow validation reports | Mapping only. |
| `archive/` | superseded or historical reports, including defense-era material | Mapping only; nothing archived yet. |

When a later phase moves a document, it must retain the filename, update every
repository Markdown link, and leave a compatibility notice at the old path.
