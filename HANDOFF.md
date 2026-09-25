# Handoff – 2026-09-25 session

Goal: fix `getElem() should not see invalid handles` → `REQUIRED process [move_base_flex-7] has died`,
then reduce RAM/CPU pressure on the Pi Zero 2 W. Nothing from this session is committed yet.

Backup taken before any change: `/opt/open_mower_ros_backup_20260925_180806.tar.gz` (whole workspace incl. .git).

## 1. MBF crash – root cause and fix

- The message comes from actionlib's client-side `ManagedList::Handle::getElem()`, not MBF itself.
- MBF 0.4.0 `MoveBaseAction::replanningThread()` polled `action_client_get_path_.getState()` every 5 ms
  (even with replanning off) while `start()` (new move_base goal) / `cancel()` / client callbacks used the
  same `SimpleActionClient` on other threads. `gh_.reset()` between the `active_` check and `getElem()`
  → invalid iterator → segfault. Hit on every drive to a mow path's first point; slow Pi widens the window.
  Upstream MBF (noetic/master) never fixed it.
- Fix:
  - Vendored MBF 0.4.0 `mbf_abstract_nav` into `src/lib/mbf_abstract_nav/`; patched `replanningThread()`
    to only touch the clients for replans it sent itself. Tests made opt-in in its CMakeLists (no rostest in sysroot).
  - `src/open_mower/params/move_base_flex.yaml`: `planner_frequency: 0.0` (costmaps are static-map only,
    replanning gave nothing).
- Reverted the previous AI's crash-survival work (commit e0f35cf), which caused lock-ups:
  - `_move_base.launch`: MBF back to `required="true"` (was respawn).
  - `MowingBehavior.cpp`: removed `mbfDied`/`waitForMbfRespawn`/`MBF_HANDOVER_SETTLE` and the resume blocks.
    Kept its `ensureServerReady`/`cancelAndDrain`/`sendGoalAndConfirm` helpers.
- Verified on the Pi that MBF loads the patched lib from `devel/lib`. **Not yet verified under real mowing.**

## 2. RAM / CPU reductions

| Change | Where | Saves |
|---|---|---|
| Removed `move_base_legacy_relay` (unused) | `_move_base.launch` | ~49 MB |
| Gamepad teleop include commented out (app is used instead) | `open_mower.launch` | ~40 MB |
| Old IMU stack no longer launched (see §3) | `open_mower.launch`, `imu.launch` | ~11 MB |
| xbot_monitoring: `getTopics()` 10 Hz → 1 Hz, lock around `active_subscribers` (thread race) | `src/lib/xbot_monitoring/src/xbot_monitoring.cpp` | ~10% CPU on rosmaster |
| `gpu_mem=16` (user, in `/boot/firmware/usercfg.txt`, applied after reboot) | Pi | 48 MB |
| Disabled bluetooth, hciuart, networkd-dispatcher, avahi (user) | Pi | ~15 MB |

Result 6 min after reboot: MemTotal 455 MB (was 407), available 156 MB (was ~110–125),
SD swapfile 0 MB (was 107 MB), memory pressure full avg10 0.8% (was 9.6%). Remaining swap is zram only.

Not done: map resolution stays 5 cm (user requirement). `str2str` (2 procs, outside ROS) uses ~11% CPU – unreviewed.

## 3. IMU didn't start from roslaunch

- `src/lsm6dsv_imu/` (package.xml) was never synced to the Pi → roslaunch "cannot launch node"; rosrun worked
  via `devel/lib` fallback. User had been starting it manually with `start_imu.sh`. Now synced; the manual
  script is no longer needed.
- `open_mower.launch` line 2 `#<include .../imu.launch/>` and all `#` lines in `imu.launch` were **active**
  (XML has no `#` comments) → mpu9255_node, complementary_filter_node (publish_tf=true) and a second
  lsm6dsv driver were started. Converted to real `<!-- -->` comments.
- Local `open_mower.launch` was rebased on the Pi's version (it had the lsm6dsv node and `coverage_planner`).

## 4. worx_comms segfault after the Release build

- Release (-O3, NDEBUG) exposed UB that -O0 tolerated. Fixed in `src/worx_comms/src/worx_comms.cpp`:
  - rx loop read `rxBuffer[250]` (`<=` → `<`),
  - unbounded `rxmsgBuffer[rxPos++]` when an EOF byte is lost → now drops with `SPI rx message too long, dropping`,
  - tx `strncpy`/`strlen` overrun → bounded `memcpy`,
  - `document.IsObject()` / `IsObject()` / `IsString()` checks before rapidjson access.
- Build switched to `RelWithDebInfo` (-O2 -g) so any new crash can be symbolized.
- Running on the Pi since the last deploy; not yet tested under mowing.

## 5. slic3r_coverage_planner removed as a build dependency

- It has CATKIN_IGNORE; mower_logic only compiled thanks to stale generated headers in the old `devel/`.
  A clean rebuild (needed after a WSL VM crash truncated 55 build artifacts) exposed it.
- mower_logic and mower_utils now use `coverage_planner`'s identical `PlanPath`/`Path` types (CMakeLists,
  package.xml, includes, `coverage_planner::` namespace). Service name string `slic3r_coverage_planner/plan_path`
  is unchanged – coverage_planner advertises that name, so it's wire-compatible.

## 6. FTC: turn froze after a path end ("motor controllers disabled")

- Symptom: after a mow path ended, the FIRST POINT drive sent no wheel commands; MBF aborted for
  "oscillation" after 10 s, the retry timed out in PRE_ROTATE (20 s). worx_comms/board were fine
  (no emergency, no MOTORREQ_DISABLE) – FTC simply output zero.
- Cause: FTC's oscillation detector was never reset in `setPlan()`, and on oscillation in PRE/POST_ROTATE
  a local change (pre-absorb, reason unknown; upstream spun at `+max_cmd_vel_ang`) set angular to 0.
  A flag from the previous path's POST_ROTATE froze the next turn. Its buffer is sized for 10 Hz but
  MBF runs FTC at `controller_frequency: 1.0`, so it needed ~50 s to clear.
- Fix (`src/lib/ftc_local_planner`): reset oscillation state + detector in `setPlan()`; replaced the zeroing
  with a **turn assist**: an oscillating or stalled turn (<5° progress in 3 s) turns toward the target at
  0.4 rad/s while reversing at 0.05 m/s, max 12 cm per rotate phase (constants `kTurn*` at the top of
  `ftc_planner.cpp`). Deployed 2026-09-25, not yet tested under mowing.
- Consider later: `controller_frequency: 1.0` is low for turning (large overshoot per cycle); try 2–5 Hz
  if CPU allows.

Note – three different "oscillation" checks: (1) FTC `oscillation_recovery` detector (flipping turn
commands; the cause, fixed here), (2) upstream #167 disables (1) entirely (not applied), (3) MBF
`oscillation_timeout`/`oscillation_distance` = "robot moved < 0.2 m in 10 s" watchdog; its message
"The controller is oscillating" is a symptom, and long in-place turns can trip it.
Detector detail: with `oscillation_v_eps`/`omega_eps` = 5.0 (cfg defaults; values are normalized to ±1) the
mean checks always pass, so "oscillating" = ≥2 sign changes of the turn command in a 50-sample buffer
(~50 s at 1 Hz) that was never cleared between goals → hair-trigger; any turn >5 s
(`oscillation_recovery_min_duration`) then froze. Cause identified by elimination (FTC ran, PWM 0, flag was
set, only zeroing path) – the new build logs `turn oscillating/not progressing` if it triggers again.

## 7. Upstream (ClemensElflein/open_mower_ros) – what we're missing

`upstream` = official Clemens repo, `origin` = Damme fork. Fetched 2026-09-25: fork diverged at
`c9c8492` (2024-05-13); `upstream/main` is 933 commits ahead (latest 2026-09-07). Local == `origin/worx_claude`.

Relevant to today's issues (not applied yet – evaluate first):

| Upstream | What | Relevance |
|---|---|---|
| `da66242` #167 (2025-01) | `oscillation_recovery: false` in ftc/docking/sim ftc yaml ("rotate on the spot collides with obstacles") | Same code path that froze our turn. Suggested: adopt it; our turn assist still catches stalled/overshooting turns via the "no angular progress" check. One yaml line, no rebuild. |
| `c95dcf9` #294 (2026-06) | `BackwardForwardRecovery` MBF recovery plugin (obstacle look-ahead) + mower_logic runs recovery behaviors; `recovery_behaviors:` / `recovery_enabled: true` in move_base_flex.yaml | Upstream's version of "back up when stuck". We log `No Recovery Behaviors loaded!`. Needs careful merge (touches FTC + MowingBehavior, both locally modified). |
| `26c1e4e` #159 (2025-06) | costmap params ported to post-Hydro format | Fixes flaky/empty costmaps from mixed pre/post-Hydro params; ours still use `static_map: true`. |
| MBF getElem race | **Not fixed upstream** (they use MBF `respawn="true"`, `planner_frequency: 1.0`, `sleep(1)` after sendGoal) | Our vendored MBF patch is ahead – candidate for an upstream PR. |
| `controller_frequency` | still 1.0 upstream (was 5.0 until upstream `a4c6abf`, 2022-03, set for the pre-FTC planner) | Optional experiment: 5.0 in move_base_flex.yaml (no rebuild). |

Remaining ~900 upstream commits are mostly xbot_framework, OS/RAUC image, MQTT, simulation – not triaged.

## 8. GPS loss reaction (was ~11 s) + blade toggling – 2026-09-25 late

- `xbot_positioning` kept FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE + the last good fix's accuracy for a
  hard-coded 10 s after the last ACCEPTED fix, so `OM_GPS_TIMEOUT_SEC` only started counting after that
  (log: GPS stop 1832.9 -> "Low quality GPS" 1843.0). Now param `recent_gps_timeout`
  (launch: `OM_GPS_RECENT_TIMEOUT_SEC`, default 1.5 s) and the flag also requires `has_gps`
  (after >5 s without GPS the pose counts as GPS-backed only after 10 re-accepted fixes).
- `mower_logic.cpp checkSafety`: blade was enabled at the top of every 0.5 s tick and stopped again by the
  GPS-timeout / stale-pose checks -> on/off every 0.5 s. Enable moved after those checks.
- `MowingBehavior`: GPS was only checked between paths; the running exe_path kept driving (mower_logic's
  0.5 s zero on logic_vel vs twist_mux 0.5 s timeout leaks). Now FIRST POINT and MOW loops cancel + pause on
  `!hasGoodGPS()` (not counted as a first-point attempt), resume from the progress index, and wait
  `gps_wait_time` after GPS returns. Pause wait loop now also exits on abort.
- Built, not yet tested on the mower. Test: pull the NTRIP/antenna during mowing, expect stop in ~2-3 s.

## Status at end of session (2026-09-25 evening)

Deployed and running on the Pi: MBF patch + planner_frequency 0, RelWithDebInfo build, worx_comms fixes,
xbot_monitoring fix, IMU/launch fixes, memory reductions. Mower is charging.
FTC oscillation reset + turn assist (§6) deployed 19:44 and verified loaded (md5 match, MBF restarted after copy).

Plan: user evaluates tomorrow. If problems remain, continue in a new session with the logs:
`~/.ros/log/latest/` on the Pi (`rosout.log`, `roslaunch-*.log`; worx_comms `>>>`/`<<<` lines are
throttled to 1/s, so they cannot show rates above 1 Hz).

## Changed files (uncommitted)

`src/lib/mbf_abstract_nav/` (new), `src/open_mower/params/move_base_flex.yaml`,
`src/open_mower/launch/open_mower.launch`, `src/open_mower/launch/include/{_move_base,imu}.launch`,
`src/mower_logic/{CMakeLists.txt,package.xml}`, `src/mower_logic/src/mower_logic/{mower_logic.cpp,behaviors/MowingBehavior.{h,cpp}}`,
`src/mower_utils/{CMakeLists.txt,launch/planner_test.launch,src/planner_test.cpp,src/xbot_pose_converter.cpp}`,
`src/worx_comms/src/worx_comms.cpp`, `src/lib/xbot_monitoring/src/xbot_monitoring.cpp` (submodule),
`src/lib/ftc_local_planner/src/ftc_planner.cpp`, `src/lib/ftc_local_planner/include/ftc_local_planner/ftc_planner.h`.
Pre-existing user change, not from this session: `src/lsm6dsv_imu/src/lsm6dsv_imu_node.cpp`.

## Next steps

1. Mow a few sessions; watch for `getElem()` / `move_base_flex ... has died`, worx_comms crashes,
   `SPI rx message too long` warnings. Test first-point drives, pause, skip area/path, docking.
2. If something segfaults: `sudo dmesg | grep segfault` on the Pi → map the offset with
   `aarch64-linux-gnu-addr2line -f -e devel/lib/<pkg>/<bin> <offset>` locally.
3. Commit the work in logical commits (MBF vendor+patch, config, launch/IMU, worx_comms, xbot_monitoring
   (submodule!), slic3r→coverage_planner). `src/lib/xbot_monitoring` needs to be absorbed or committed in the submodule.
4. Optional: move worx_comms' packet dumps to `ROS_DEBUG` when bug hunting is over
   (toggle at runtime with `rosconsole set /worx_comms ros.worx_comms debug`); `rosclean purge` on the Pi
   (~13–19 GB of old logs); review `str2str` CPU use.
5. Remaining small MBF race: `cancel()` during the get_path→exe_path handover. If crashes persist, add
   proper locking in `MoveBaseAction` (beware: actionlib callbacks hold the client's list mutex – naive
   locking deadlocks).
