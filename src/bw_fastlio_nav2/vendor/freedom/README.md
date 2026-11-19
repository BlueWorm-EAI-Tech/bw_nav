# Vendored FreeDOM core

Source: [LC-Robotics/FreeDOM](https://github.com/LC-Robotics/FreeDOM), commit recorded in `UPSTREAM_COMMIT` (`dcfd5690cafddf121a80bd8ea477807d9656748e`). The accompanying MIT `LICENSE` is copied verbatim. Only the offline core is included; the build does not fetch upstream or generate these files.

The offline headers are based on the frozen replay's `analysis/dynamic_map_freedom/prepare_core.py`: ROS 1 includes and ROS-only helpers removed, explicit PCL headers added, and `DepthImage::learn_fov` initialized even when raycast enhancement is disabled. This copy also:

- exposes the read-only static map rather than writing PCDs, and removes timing instrumentation from the integration path;
- preserves the original signed 32-bit index hash bits using unsigned arithmetic, and uses floor division for negative hierarchical cell indices;
- uses instance-specific allocation sizes for sparse map blocks instead of mutable process-wide dimensions, and rejects scan sequence overflow before the empty-cell sentinel can be used;
- retains the upstream scan, raycast, and map classification algorithms. The caller uses one worker (the validated replay configuration).

Build the six files under `src/` as C++17 with the private `include/` search root. Dependencies: Eigen, PCL common, Threads, and OpenCV core/imgproc/imgcodecs/photo (upstream depth-image implementation). The wrapper lives outside this directory and owns the same-base mask policy.
