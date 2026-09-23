# Escort guard + GBSFLACS live simulation

The live controller uses `combined_reference.py`, whose non-visual algorithm
and state-machine section matches the supplied
`护航守卫_GBSFLACS_三维单目标_Python39-2.py`.  Matplotlib-only code is not used.

Launch the complete scenario with:

```bash
source ~/uav_usv_ros1_ws/devel/setup.bash
roslaunch uav_usv_bringup gbsflacs_332_sim.launch
```

Runtime sequence:

1. wait for all six ArduPilot agents and base-station leases;
2. issue TAKEOFF to all three UAVs in one controller cycle;
3. move all three USVs into the escort ring and hold the completed escort
   formation for 20 seconds;
4. form core, wing and support guard positions around the randomly patrolling
   threat and hold the completed guard formation for 20 seconds;
5. automatically run three-dimensional GBSFLACS capture;
6. after geometric capture, maintain a 2x3 rectangular formation centered on
   the enemy and continuously follow its random motion.

Important live parameters are in `config/heterogeneous_332_algorithms.yaml`.
The enemy starts 140.4 m from the protected ship and randomly patrols the
eastern open-water box continuously at 0.4--1.2 m/s.  It does not intentionally approach
the protected ship.  The guard USVs are limited to 10.0 m/s.  The final 2x3
formation uses 24 m spacing for vessel clearance while following the enemy.
The actual Gazebo
vessel limits are in the three `sim332_usv_*` model SDFs and
`sim332_enemy_ship/model.sdf`.
