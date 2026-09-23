# GBSFLACS ROS 1 integration

This adapter runs the supplied `围捕727+3+3+1_GBSFLACS.py` decision logic on
the live 3 UAV + 3 USV simulation.  The stored `gbsflacs_reference.py` differs
from that file only by selecting Matplotlib's headless `Agg` backend.

## Algorithm mapping

1. Live `VehicleState` rows replace the reference simulator's mutable agent
   array.  IDs 1000-1002 map to `uav_01`-`uav_03`; IDs 2000-2002 map to
   `usv_01`-`usv_03`.
2. The selected entity in `/fleet/world_model` replaces the reference target
   row.  Target motion is not predicted twice: the live world model is the
   authoritative position source.
3. GB synchronizes granular-ball membership and quality from observed poses.
4. Capacity-aware assignment and CS optimize the grouping/assignment cost.
5. SFLA optimizes ring phase, radius and UAV vertical offset.  Hungarian
   matching assigns the resulting heterogeneous slots to vehicles.
6. The adapter converts each final slot into bounded FleetCommand segments.
   UAV segments are at most 8 m by default, avoiding the known long-waypoint
   limitation in the current ArduCopter agent.

The reference simulator's direct `self.agents[...] = ...` motion update and
Matplotlib GUI are not used by the ROS adapter.

## Safety and ownership

- The node only publishes `COMMAND_NAVIGATE` on `/fleet/command`.
- It never publishes `/fleet/control_lease`; an existing valid lease is
  required for every vehicle.
- It never arms, takes off, changes Gazebo poses, connects to MAVLink, or sends
  motor commands.
- A vehicle receives no new segment while its prior command is active.
- Rejected/failed/canceled commands and terminal-ACK timeouts stop planning.
- `auto_start` and the main-launch `start_gbsflacs` switch both default false.

## Start

Start the simulation and the dormant adapter:

```bash
roslaunch uav_usv_bringup heterogeneous_332_qt_ros1.launch \
  start_ardupilot:=true start_algorithms:=false start_gbsflacs:=true
```

First use the existing FleetCommand workflow to take all UAVs off safely.
Then press the Qt `切换GBSFLACS围捕` button, or publish:

```bash
rostopic pub -1 /fleet/algorithm/action std_msgs/String \
  "data: 'CAPTURE:enemy_ship'"
```

Observe `/fleet/gbsflacs/status`, `/fleet/gbsflacs/plan`,
`/fleet/command_ack`, and `/fleet/state` during execution.
