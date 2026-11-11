# MID360 PGO, 3D localization and Nav2

## Mapping

```bash
source install/setup.zsh
ros2 launch bw_fastlio_nav2 mid360_pgo_mapping.launch.py rviz:=true
```

The mapping launch keeps a live obstacle preview while FAST-LIO and the PGO
backend build the 3D map:

```text
/cloud_registered_body (10 Hz) -> voxel -> Patchwork++
  -> /ground_points_live + /obstacle_points_live -> /obstacle_grid_live
```

This live output is not baked into the global map. The static map is generated
offline after PGO so transient objects from individual scans do not become
permanent obstacles.

After completing a loop, stop the robot and run the full pose-graph solve:

```bash
ros2 service call /pgo_batch_optimize std_srvs/srv/Trigger '{}'
```

Rebuild the optimized 3D map from the original keyframes. A voxel must be
observed by at least two different keyframes, which removes single-frame noise
and most short-lived objects:

```bash
ros2 run bw_fastlio_nav2 rebuild_pgo_consistent_map \
  --scans-directory maps/pgo_session/Scans \
  --poses maps/pgo_session/optimized_poses_batch.txt \
  --output maps/pgo_session/map_consistent.pcd \
  --voxel-size 0.05 --min-observations 2 --max-range 60
```

The defaults bake in the settings that were measured to matter on MID360 data:

| option | default | why |
| --- | --- | --- |
| `--voxel-size` | `0.05` | 0.15 m bins merge 20 observations each and halve the map's surface resolution; 0.05 cuts plane thickness from 22.8 mm to 11.8 mm (p50) |
| `--z-min` / `--z-max` | `-0.60` / `2.50` | drops ceiling returns (the ceiling sits at z≈3.1 m) and stray points below the floor; costs ~27% of raw points and changes nothing inside the band |
| `--min-observations` | `2` | per-voxel count of *distinct keyframes*; on dense data it only trims ~2% of voxels, so it is not the density control |
| `--normal-voxel-size` | `0.30` | normals are estimated on a coarser grid than the output voxels so each estimate has enough points |
| `--min-planarity` | `0.55` | `(λ1-λ0)/λ2` gate; cells below it are treated as unconstrained and get weight 1.0 instead of being penalised |
| `--max-incidence-deg` | `88` | grazing returns are discarded above this incidence angle. 88° means "essentially never reject" — measured, hard rejection at 60° gains 6% thickness while losing 9% XY coverage |
| `--full-weight-deg` / `--min-weight` | `60` / `0.10` | incidence weighting inside a voxel; keeps a grazing return's vote at 1/10. Measured effect is small because a 5 cm voxel barely spans any viewing-angle variation |
| `--threads` | `0` (= all cores) | scans are spread over a fixed worker pool with atomic work stealing; output is sorted by (x, y, z) so 1 thread and 16 threads produce byte-identical files |

Both passes over the keyframe directory are parallel, and each worker keeps a
private hash map that is merged afterwards, so the result does not depend on the
scheduling order.

Build the matching two-dimensional maps from the optimized PCD with CSF cloth
simulation filtering:

```bash
ros2 launch bw_fastlio_nav2 generate_nav2_maps.launch.py
```

The launch defaults are the tuned production recipe and write two maps:
`mid360_pgo_csf` for Nav2 and `mid360_pgo_bbs` for the BBS relocaliser.  The
nav map groups each object's returns with a 0.10 m disk closing, removes
isolated blobs below 20 cells, ray-casts free space from the trajectory with a
4 m geodesic cap, keeps wall-enclosed rooms as unreachable free islands and
seals every remaining free/unknown boundary with a wall.  The BBS map keeps the
raw obstacle layer (no clustering, no component filter) and uses a cheap
0.08 m euclid free layer without sealing.  Wall bridging is off in both: on
dense recordings it chains unrelated clutter dots into long thin synthetic
walls instead of repairing real gaps.

The equivalent single-map CLI invocation is:

```bash
ros2 run bw_fastlio_nav2 pcd_to_nav2_map \
  --input maps/pgo_session/map_consistent.pcd \
  --output-prefix maps/nav2/mid360_pgo_csf \
  --poses maps/pgo_session/optimized_poses_batch.txt \
  --voxel-size 0.08 \
  --cloth-resolution 0.20 --rigidness 3 --iterations 500 \
  --classification-threshold 0.18 --time-step 0.65 \
  --slope-smoothing false \
  --min-height 0.25 --max-height 1.80 \
  --ground-reference-radius 0.75 --ground-reference-neighbors 5 \
  --outlier-radius 0.20 --outlier-min-neighbors 6 \
  --resolution 0.05 --padding 1 --occupied-radius 0 \
  --min-obstacle-neighbors 2 --wall-bridge-length 0 \
  --obstacle-cluster-radius 0.10 --min-obstacle-component-cells 20 \
  --free-radius 0.4 --free-mode raycast --free-max-range 40 \
  --free-max-offset 4 --free-islands enclosed \
  --free-island-wall-ratio 0.4 --free-island-max-area 25 \
  --min-free-component-cells 100 --seal-mode free --unknown-value 127
```

CSF estimates a terrain surface over the complete PGO cloud, so no fixed
`floor-z` or single global plane is used. `cloth-resolution` controls terrain
detail, while `classification-threshold` is the maximum cloth distance still
classified as ground. Enable `slope-smoothing` for ramps or steep terrain; keep
it disabled for a mostly level indoor floor. Each off-ground point is compared
with nearby CSF ground before the `min-height`/`max-height` obstacle window is
applied.

The offline default starts static obstacles at `0.25 m` above the local CSF
terrain to keep floor residue out of the permanent map. Objects below that
height are still handled online by Patchwork++, whose dynamic threshold remains
`0.10 m`.

## Localization and Nav2

Stop `mid360_pgo_mapping.launch.py` before starting localization. Mapping and
navigation each start CPU-intensive point-cloud components and must not run at
the same time on the robot IPC.

The first run must remain disarmed:

```bash
source install/setup.zsh
ros2 launch bw_fastlio_nav2 mid360_nav2_localization.launch.py \
  enable_motion:=false \
  body_to_base_x:=-0.1 body_to_base_y:=0.0 body_to_base_z:=-0.35 \
  body_to_base_roll:=0.0 body_to_base_pitch:=0.0 \
  body_to_base_yaw:=0.0
```

The launch uses the PGO PCD for NDT_OMP tracking, the corresponding occupancy
map for Nav2 and BBS global search, and FAST-LIO for the live
`camera_init -> body` odometry edge. The required TF tree is:

```text
map -> camera_init -> body -> base_footprint
```

The measured sensor pose is `base_footprint -> body = (0.10, 0.0, 0.35) m`.
The sensor and chassis axes are aligned: `+X` forward, `+Y` left and `+Z` up.
The inverse TF is therefore
`body -> base_footprint = (-0.10, 0.0, -0.35) m` with `yaw=0`.

FAST-LIO publishes pose-only `/Odometry`; its twist is zero. The
`fastlio_planar_odom` node composes the sensor extrinsic, differentiates the
`base_footprint` pose and publishes filtered `/nav2_odom`. Nav2 and the chassis
command bridge use that topic for closed-loop speed feedback. The previous V2
chassis response calibration is temporarily reused on V3. The bridge converts Nav2's desired physical speed
through separate measured lookup tables for X/Y positive and negative motion,
then raises a nonzero raw command to `0.427` only while measured speed
indicates that the chassis is stationary. Nav2 therefore plans in the measured
speed domain while the calibrated output crosses the breakaway dead zone.

The `velocity_passthrough` launch argument (default `true`) switches all of that
shaping off: `/cmd_vel_safe` carries the incoming `/cmd_vel` verbatim, with no
calibration inverse map, breakaway boost, lateral suppression, rear-turn
alignment, goal-yaw rotation or output clamp. Only the safety interlocks remain
active, so a stale command, an unpowered chassis, or stale localization/odometry
still forces a zero command. Pass `velocity_passthrough:=false` to restore the
shaping chain.

Use RViz **2D Pose Estimate** for a known initial pose. With
`auto_relocalization:=true`, startup can also use BBS coarse search followed by
3D NDT scoring; verified poses are saved and checked against the PCD hash before
reuse. Lost tracking raises a guarded global relocalization request.

Check the stationary system before enabling chassis output:

```bash
ros2 topic hz /cloud_registered_body
ros2 topic hz /obstacle_points_live
ros2 topic hz /obstacle_points_fov
ros2 topic echo /localization/pose_with_covariance --once
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /cmd_vel --once
```

The NDT tracker is capped at 5 Hz because registration reached about 0.20
seconds under concurrent mapping load. Poses and `map -> camera_init` keep the
accepted scan timestamp; they are not re-stamped with wall time.

Nav2 uses the CSF occupancy map as its static bottom layer. Patchwork++ runs on
each `/cloud_registered_body` scan and publishes the ground/self-filtered
`/obstacle_points_live`. That cloud is then trimmed by a field-of-view gate and
published as `/obstacle_points_fov`, which feeds the `obstacle_layer` of both
the global and the local costmap above the static map. The gate keeps only
returns at or below the LiDAR plane (`fov_min/max_elevation_deg`, default
-90..0 deg) and inside +-135 deg of the forward axis
(`fov_min/max_azimuth_deg`), so the rear quarter and overhead returns never
reach a costmap while `/obstacle_points_live` keeps the full filtered cloud.
Each costmap source keeps the 2D thresholds (0.05 m cells, 8 m range,
0.5 s observation persistence); transient objects never modify the static map.

All gate limits are exposed by `mid360_nav2_localization.launch.py`:
`fov_filter_enabled`, `fov_obstacle_topic`, `fov_min_azimuth_deg`,
`fov_max_azimuth_deg`, `fov_min_elevation_deg`, `fov_max_elevation_deg`,
`fov_min_range`, `fov_max_range` and the limiting-frame orientation
`fov_mount_roll_deg`, `fov_mount_pitch_deg`, `fov_mount_yaw_deg`. For example,
`fov_max_azimuth_deg:=180.0 fov_min_azimuth_deg:=-180.0` restores a full-circle
gate. `collision_monitor` keeps consuming the unfiltered
`/obstacle_points_live`.

Only after the PCD alignment, TF direction, footprint, global/local costmaps and
planned path are correct:

```bash
ros2 launch bw_fastlio_nav2 mid360_nav2_localization.launch.py \
  enable_motion:=true protocol_version:=v3 robot_version:=2.0
```

Nav2 is limited to measured speeds of `0.30 m/s` in X, `0.10 m/s` in Y and
`0.25 rad/s` in yaw. The lower lateral limit and stronger forward/path-alignment
costs keep the front-mounted MID360 facing approximately along the direction of
travel. The V2 bridge maps translation through direction-specific lookup tables,
without an additional linear scale. The yaw raw
command is multiplied by `0.538` because direct feedback showed an angular gain
of about `1.86`. Translation below `0.02 m/s` is zero; other nonzero translation
uses a `0.427` raw-command breakaway while the chassis is stationary.
The bridge consumes Nav2's final `/cmd_vel` output so path tracking uses the
open-loop velocity smoother and recovery commands also reach the chassis.
The controller is Nav2's `nav2_mppi_controller::MPPIController` with the omni
motion model. MPPI samples its default `2000` rollouts over `56` prediction
steps, scores them with six critics and applies the cost-weighted average of the
best candidates. A `temperature` of `0.25` sharpens that weighting so the
lowest-cost rollout wins decisively instead of the controller blending
near-equal candidates, and `gamma: 0.015` keeps the remaining jitter in check.
Speed limits are `0.30 m/s` forward, `0.10 m/s` lateral and `0.25 rad/s` yaw, and
`vx_min: 0.0` still rules out reverse trajectories. Humble's MPPI exposes no
acceleration limits, so the former DWB `0.80 / 0.80 / 0.40` acceleration and
deceleration envelope is enforced by the open-loop velocity smoother.
`GoalCritic` (`25.0`) dominates inside `4.0 m` so the approach does not
degenerate into an arc, while `PathFollowCritic` and `PathAlignCritic` keep the
front of the robot facing the direction of travel. Fixed positive recovery spins
are disabled. Rear-path alignment follows the shortest signed heading error after
replanning, retaining its previous direction only inside a narrow `0.05 rad` tie
region around pi. Lateral commands below `0.08 m/s` are converted into
turn-then-forward motion at the hardware boundary:
the chassis only stops for alignment above `0.35 rad`; otherwise it sends the
command magnitude as forward speed while turning. The full `0.10 m/s` lateral command remains available when MPPI
needs a decisive obstacle sidestep. Within `0.30 m` of the goal, a pure final
yaw adjustment retains normal turn speed while the error exceeds `30 deg`, then
uses the shortest signed error at `0.08 rad/s` for fine adjustment and stops
inside the configured `0.10 rad` tolerance to avoid overshoot-driven oscillation.
Global planning uses Smac 2D with a `4.0` traversal-cost multiplier. Both
costmaps retain a `0.50 m` inflation radius with a slower `0.6` cost decay, and
`ObstaclesCritic` adds a repulsion term on top of the costmap gradient, so an
open detour is preferred over a shorter path along an obstacle whenever the map
offers one.
It publishes `/yaw_tracking_status` and logs the final-plan yaw, localization
yaw, normalized yaw error, plus desired/raw/measured X, Y and yaw velocities.
The bridge requests whole-robot software power once, when the first valid
nonzero navigation command arrives. It never powers the whole robot off during
normal stops, command timeouts, relocalization, localization/odometry faults or
shutdown. Those paths send zero chassis velocity instead, while the serial
layer's 250 ms command watchdog remains active. A localization jump holds zero
velocity and permits one guarded automatic recovery between manual pose resets:
alignment must remain converged and healthy, ten consecutive poses must remain
continuous for at least five seconds, odometry must be fresh, and measured
chassis speed must be stationary. A second jump stays latched until an accurate
RViz **2D Pose Estimate** is supplied. Competing downstream `Teleop/cmd_vel`
publishers remain latched until restart. The continuous-rotation watchdog re-arms
only after the rotation request has stopped for five seconds.

For V3, the serial node requires a valid feedback frame before accepting the
software-power request. It first copies the current pelvis, arm, waist and head
positions into the outgoing command and clears chassis velocity, joint velocity
limits and torque commands so enabling navigation does not command a zero-pose
jump.

The live body-frame Patchwork++ separator uses the measured `0.15 m` sensor
height, a `0.05-15.0 m` processing range, a `0.02 m` patch distance threshold,
and a `0.10-1.80 m` obstacle-height window above the nominal floor, followed by
the field-of-view gate described above. Reflected noise removal is disabled
because MID360 intensity is not on the scale assumed by the upstream default.
Radius filtering defaults to `0.20 m` and two neighbors. If RViz shows real
objects being removed or excessive isolated noise, tune the final radius filter
without rebuilding:

```bash
ros2 launch bw_fastlio_nav2 mid360_nav2_localization.launch.py \
  enable_motion:=false \
  live_outlier_radius:=0.20 live_outlier_min_neighbors:=3
```
