# Legacy Interface Inventory

> Phase 1 inspection only. No topic, frame, parameter, or executable is changed.

| Reference | Current classification | Evidence | Required future action |
|---|---|---|---|
| `/fleet/*`, `/perception/*`, `/base_station/*` | canonical | Current 332 Fleet World Model, Base Station Service and perception architecture | Keep frozen. |
| `/maritime/tracks/{lidar,camera,ais,fused}` | documentation-only stale reference | `docs/INTERFACES.md` and older perception documentation | Mark legacy when documentation is rehomed; do not rename a runtime producer in Phase 1. |
| `/maritime/ais/*`, `/maritime/ownship/odom` | compatibility alias | Original `uav_usv_sim` AIS/COLREG scenario scripts | Retain for compatibility scenarios; classify after COLREGs package becomes active. |
| `landing_boat/*` frames and parameters | compatibility alias | Original Nav2, RViz, SDF, URDF and mission demos | Retain in `uav_usv_sim`; do not alter current 332 `map -> <vehicle>/base_link` chain. |
| `landing_boat/hull/front_lidar` | compatibility alias | Original boat SDF, Nav2 config and legacy demos | Retain only in old world flow; no global rename. |
| `maritime_tf_publisher` | compatibility alias | Original simulation install/launch entry | Keep until production localization owns the corresponding flow. |
| `enable_legacy_topic_fallback` | canonical compatibility control | Fleet gateway config and gateway node | Keep; it explicitly protects old clients without becoming a second canonical source. |
| `legacy_uav` / `legacy_usv` parameters | compatibility alias | Capture manager accepts old singular settings | Keep to preserve old launches; do not remove while they are runnable. |
| `within_maritime_watch_area` | canonical internal label | Fleet World Model reason text | No topic/frame migration needed. |

## Result

No active 332 interface is approved for renaming. Historical maritime and
`landing_boat` names are confined primarily to retained compatibility worlds,
Nav2/AIS/COLREGs demonstrations and old documentation. They must be migrated
only with a topic/frame contract test and a compatibility release note.
