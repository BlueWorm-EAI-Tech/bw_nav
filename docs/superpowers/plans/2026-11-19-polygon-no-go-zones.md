# Polygon No-go Zones Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement and verify in order.

**Goal:** Edit persistent polygon planning zones through a standalone GUI or ROS services.

**Architecture:** `bw_interface` defines the management contract. `bw_planning_zones` is the sole online writer and publishes complete mask, marker and state snapshots. `bw_fastlio_nav2` applies the mask to its global costmap and displays the markers in its existing RViz config.

**Tech Stack:** ROS 2 Humble, Python/Tk, Pillow, Shapely, C++17/Nav2.

**Spec:** `docs/superpowers/specs/2026-11-13-polygon-planning-zones-design.md`

## Global Constraints

Only traversable global cells become 252; preserve 253–255 and do not modify the local costmap, PGM or motion pipeline. 252 remains traversable. Work on `feat/polygon-no-go-zones`; exclude existing recovery-layer working changes from commits.

## Review Focus

Concave notches and boundary contact; stale revisions and external edits; deletion and delayed ROS subscribers; wrong map geometry; planner starting inside the zone.

### Task 1: Geometry, storage and typed services

- [x] Add tests for `MapSpec`, polygon clipping/triangulation, persistence, invalid geometry and stale revisions.
- [x] Implement `geometry.py`, `store.py` and `bw_interface` messages/services; run Python tests and build interfaces.

### Task 2: Publisher and editor

- [x] Test headless create/delete, snapshot replay and persistence; test GUI vertex editing under Xvfb.
- [x] Implement `server.py`, `editor.py`, Python packaging and `planning_zones.launch.py`; rerun tests.

### Task 3: Global integration

- [x] Add tests for 252 composition, preservation of 253–255, and replacement/removal of an applied mask.
- [x] Implement `PlanningZoneLayer` and update navigation launch, parameters and existing RViz config.
- [x] Build the three affected packages against ROS Humble; verify the global filter and isolated Smac planner with production planner parameters.
- [x] Check the existing RViz configuration against a late subscriber and mask deletion without starting chassis nodes.
- [x] Review the feature and cover draft conflicts, missing replies, distro Pillow compatibility, and grid-edge rounding.

Verification: the feature tests pass. The separate legacy navigation-motion and FOV suites have eight failing assertions reproducible on base commit `9e41b32`. No real-robot test was performed.

Ruling: Keep the existing smoothing parameters and test search and output separately. Search prefers to avoid 252; smoothing may enter a traversable zone. The integration test also exercises a route with no possible bypass and replanning after mask removal.
