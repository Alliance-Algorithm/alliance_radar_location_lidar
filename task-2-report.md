# Task 2 Fix Report

## Re-review finding: counter reset after baseline

The reader previously treated every counter value of zero as an uninitialized
wait. After a nonzero frame had been accepted, a producer reset to counter zero
was therefore silently ignored. The reader now treats `last_seen != 0` followed
by counter zero as an explicit FIFO overrun and transitions to
`ReaderState::overrun`. Counter zero while `last_seen == 0` remains an
uninitialized wait, including the race where the first nonzero observation
returns to zero before acceptance.

The contiguous `last_seen + 1` checks, preallocated frame pool, and lifecycle
cleanup behavior are unchanged. A focused pure test covers the reset boundary:
`0 -> 0` is not a reset, `7 -> 0` is a reset, and `7 -> 8` is not a reset.

## Verification

Command:

```text
g++ -std=c++23 -Wall -Wextra -Werror /tmp/raw_shm_counter_reset_check.cpp -o /tmp/raw_shm_counter_reset_check && /tmp/raw_shm_counter_reset_check
```

Output:

```text
(no output; exit status 0)
```

The focused package test could not be rebuilt in this environment. Command:

```text
source /opt/ros/jazzy/setup.bash && colcon build --packages-select radar_camera --cmake-args -DBUILD_TESTING=ON
```

Output:

```text
Could not find a package configuration file provided by "OpenVINO"
...
Failed   <<< radar_camera [0.15s, exited with code 1]
```

The existing build tree also had no `radar_camera_tests` executable, and this
host has no OpenCV `pkg-config` metadata or checked-out `hikcamera/shm.hpp`, so
the full C++ package test could not run locally.
