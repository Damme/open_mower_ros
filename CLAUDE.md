# OpenMower ROS – Worx fork (worx_claude branch)

ROS1 Noetic stack for a Worx mower running OpenMower on a **Raspberry Pi Zero 2 W**
(4 cores, 512 MB RAM, arm64, Ubuntu 20.04). Low-level board talks JSON over SPI (`worx_comms`).
IMU is an onboard LSM6DSV (`src/lsm6dsv_imu`). Path planning uses `coverage_planner`
(slic3r_coverage_planner is CATKIN_IGNOREd and no longer needed).

See `HANDOFF.md` for the latest session log, open issues and next steps.

## Machines

- **Dev host** `DammBurkWin` (WSL2): `/opt/open_mower_ros`. All building happens here.
- **Robot** `MrChoppie`: `ssh damme@10.99.99.99` (remote: `ssh -J damme@aux damme@mrchopper.wiegert.link`).
  - **Never edit anything on the Pi.** Read-only inspection (ps, logs, rostopic, cat) is fine. The user deploys
    (rsync) and changes Pi config/services themselves.
  - Runs natively (no Docker). `/opt/sysroot_arm64 -> /` symlink on the Pi, so paths match the sysroot.
  - `LD_LIBRARY_PATH` starts with `/opt/open_mower_ros/devel/lib`, so workspace-built libs override `/opt/ros`.
  - The Pi's launch files have been edited directly in the past. **Diff Pi vs local before syncing any
    launch/param file** (`ssh ... cat <file> | diff - <local file>`).

## Build (cross-compile)

```bash
cd /opt/open_mower_ros
source /opt/ros/noetic/setup.bash          # (+ devel/setup.bash if it exists)
catkin_make -j8 -DCMAKE_TOOLCHAIN_FILE=/opt/open_mower_ros/toolchain.cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

- Sysroot `/opt/sysroot_arm64` is partial (no rostest, no mbf_abstract_nav). New packages must build against it.
- Build type is cached in `build/CMakeCache.txt`; currently `RelWithDebInfo` (-O2 -g). Keep debug info so
  crashes can be symbolized (`aarch64-linux-gnu-addr2line -e <binary> <offset>`).
- Use `-j8`, not the default `-j16` (a -j16 debug build coincided with the WSL VM crashing).
- If the VM crashes mid-build, artifacts can be left 0 bytes and make won't rebuild them:
  check `find build devel -type f -size 0 -name '*.o' -o -name '*.so'`, then `rm -rf build devel` and rebuild.
- Quick compile check of one file without touching build/: take flags from
  `build/<pkg>/CMakeFiles/<target>.dir/flags.make` and run `aarch64-linux-gnu-g++ --sysroot=/opt/sysroot_arm64 ... -fsyntax-only`.

## Deploy (user runs it)

```bash
cd /opt/open_mower_ros && rsync -avzR \
  --exclude 'python3/' --exclude 'pkgconfig/' --exclude 'cmake.lock' \
  devel/lib/ <changed src files/packages> \
  damme@10.99.99.99:/opt/open_mower_ros/
```

- Exclude `devel/lib/python3/` (the vendored mbf_abstract_nav generates a python pkg that would shadow the system one).
- roslaunch finds packages via `package.xml` under `/opt/open_mower_ros/src` — a package whose `src/<pkg>/`
  folder is missing on the Pi works with `rosrun` but **fails in roslaunch**. Sync the package folder too.
- Make a gzip backup before bigger changes: `tar -czf /opt/open_mower_ros_backup_$(date +%Y%m%d_%H%M%S).tar.gz -C /opt open_mower_ros`.

## Gotchas learned the hard way

- **`#` is not a comment in launch XML.** Only `<!-- -->` is. `#<node .../>` is an active node.
- **actionlib `SimpleActionClient` is not thread-safe.** Using the same client from two threads gives
  `getElem() should not see invalid handles` + segfault (in the process that owns the client).
- MBF 0.4.0's `move_base` replanning thread raced its own action clients → MBF crash. Fixed by the vendored,
  patched `src/lib/mbf_abstract_nav` (see comment in `src/move_base_action.cpp`, `replanningThread`) plus
  `planner_frequency: 0.0`. Only a function body changed, so it stays ABI-compatible with the apt
  `mbf_costmap_nav` binary. Verify on the Pi: `grep mbf_abstract /proc/$(pgrep mbf_costmap_nav)/maps`
  must show `/opt/open_mower_ros/devel/lib/...`.
- Code that "worked" at -O0 can crash when optimized: buffer overruns and unchecked rapidjson types
  (Release/RelWithDebInfo define NDEBUG, which disables rapidjson's asserts).
- Global costmap follows the map resolution (5 cm, ~1049x1465 cells) because `static_map: true`; the
  `resolution: 0.10` in the costmap yaml is ignored. The user needs 5 cm.
- Memory is the Pi's bottleneck, not CPU. Check with `free -m`, `swapon --show`, `cat /proc/pressure/memory`
  and per-process `VmRSS`+`VmSwap` from `/proc/*/status` (RSS alone hides swapped-out memory).
- Map recording is done from the app: driving via xbot_monitoring/xbot_remote → `/joy_vel` → twist_mux,
  buttons via `mower_logic:area_recording/*` actions. No gamepad nodes needed.
- worx_comms logs a lot at INFO on purpose (ongoing bug hunting) — don't "clean it up" unasked.
- `src/lib/xbot_monitoring` is still a git submodule (shows as dirty when edited); other libs are absorbed.
