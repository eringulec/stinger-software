# Stinger velocity controller: Docker build and tests

Changes target `eringulec/stinger-software`, branch `codex/complete-velocity-pid`.
All seven controller exercises are implemented. The node keeps the tutorial's
publisher/subscription names, topics, callbacks, gain/state attributes, and six-item
return tuple. Surge uses body-frame `linear.x`; yaw uses `angular.z` (yaw rate,
not a heading angle). Both use commanded minus measured velocity and PID feedback.

## What has been verified

Eight standalone logic tests pass. The full ROS 2 Jazzy tutorial autograder was
also run in the user's Docker container on November 6, 2026: all 18 Q5.1.a-g
checks passed. Surge RMSE was 0.0129757 m/s and yaw RMSE was 0.0276548 rad/s,
both below 0.075. The verified gains are surge P=20, I=5, D=1 and yaw P=20,
I=8, D=1. This verifies the tutorial's combined 0.3 m/s surge / 0.2 rad/s yaw
scenario in simulation, not physical boat performance or all operating conditions.

The localization launch now remaps navsat's `imu` and `gps/fix` inputs to the
Stinger sensor topics and enables simulation time on navsat and the IMU
republisher. Before correction, filtered surge was zero while the simulated boat
was moving; after correction, filtered and ground-truth forward speeds agreed.
Original container files were backed up under
`/home/ros-tutorial/colcon_ws/controller_test_backups/20261106-213419/`.
The user-supplied EKF configuration was preserved.
Sources inspected: the fork's README, INSTALL, controller setup/launch files,
vehicle simulation, spawn launch, bridge config and throttle allocator, plus the
linked [ROS Tutorial](https://github.com/gt-marine-robotics-group/ROS_Tutorial/blob/main/README.md),
[qix stack](https://github.com/gt-marine-robotics-group/ROS_Tutorial/blob/main/qixstack.toml),
and [Topic 5.1 grader](https://github.com/gt-marine-robotics-group/ROS_Tutorial/blob/main/autograder/autograder/topic_5/question_5_1_grader.py).
The tutorial stack is ROS 2 Jazzy; the fork's older INSTALL mentions Humble.
Use the container's Jazzy installation for this workflow.

## 1. Enter the existing tutorial Docker environment

On your laptop, start Docker Desktop (Mac); on Ubuntu/WSL, use
`sudo service docker start`. From one directory above `ROS_Tutorial`:

```bash
qix stack enter ros-tutorial
```

Repeat that command in each new laptop terminal you need. Inside the container,
run this at the beginning of every terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ros-tutorial/colcon_ws
source install/setup.bash
```

If the stack has never been installed, follow the tutorial's OS-specific setup
first. In `ROS_Tutorial/qixstack.toml`, use your fork and this branch:

```toml
stinger-software = { server = "github.com", namespace = "eringulec", name = "stinger-software", version = "codex/complete-velocity-pid" }
```

From one directory above `ROS_Tutorial`, install with:

```bash
qix stack install ROS_Tutorial --novnc
qix stack enter ros-tutorial
```

The tutorial's Linux install uses `qix stack install ROS_Tutorial` (optionally
`--gpu`). With `--novnc`, the desktop is at
[localhost:8080/vnc.html](http://localhost:8080/vnc.html).
Do not reinstall an existing stack just to update controller code.

## 2. Get the branch and build (inside the container)

```bash
cd /home/ros-tutorial/colcon_ws/src/stinger-software
git remote get-url origin
git status --short
```

Confirm origin is `eringulec/stinger-software`, and save any existing edits before
switching branches. If your source folder has a different name, use that folder.
Then:

```bash
git fetch origin codex/complete-velocity-pid
git switch --track origin/codex/complete-velocity-pid
```

If already on that branch, use `git pull --ff-only` instead of `git switch`.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ros-tutorial/colcon_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables stinger_controller
python3 -m unittest discover -s src/stinger-software/stinger_controller/test -p test_velocity_controller_math.py -v
```

Expect `stinger_controller velocity_controller` and eight passing logic tests.
The README's `velocity_controllor` spelling is incorrect.

## 3. Deterministic ROS topic smoke test (no Gazebo)

Use three container terminals, sourcing the workspace in each as above. Use the
same isolated domain in all three; do not run simulation/autograder in this domain.

Terminal A:

```bash
export ROS_DOMAIN_ID=77
ros2 run stinger_controller velocity_controller --ros-args \
  -p Kp_surge:=10.0 -p Ki_surge:=0.0 -p Kd_surge:=0.0 \
  -p Kp_yaw:=5.0 -p Ki_yaw:=0.0 -p Kd_yaw:=0.0
```

Terminal B:

```bash
export ROS_DOMAIN_ID=77
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.2}}'
```

Terminal C (background odometry stays in this terminal and is cleaned up on exit):

```bash
export ROS_DOMAIN_ID=77
ros2 topic pub --rate 20 /odometry/filtered nav_msgs/msg/Odometry \
  '{header: {frame_id: odom}, child_frame_id: base_link, twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}}' &
ODOM_PID=$!
trap 'kill "$ODOM_PID" 2>/dev/null' EXIT
ros2 topic info /cmd_vel --verbose
ros2 topic info /odometry/filtered --verbose
ros2 topic echo /cmd_wrench
```

After command discovery, expect repeated `force.x: 2.0`, `torque.z: 1.0`,
`frame_id: base_link`. The first odometry sample primes the clock.
In B, Ctrl-C and publish the same command with both values negative: expect
`-2.0` and `-1.0`. Publish both zero: expect both outputs zero.
Stop all three terminals with Ctrl-C. In C, also run:

```bash
kill "$ODOM_PID"
trap - EXIT
```

This verifies real ROS messaging without depending on localization or Gazebo.

## 4. Full tutorial autograder

Stop the smoke test and any simulation/controller processes. In a fresh container
terminal, source ROS and the workspace, then:

```bash
unset ROS_DOMAIN_ID
ros2 launch autograder test_5_1.launch.py
```

This launch starts the grader and vehicle simulation. The grader itself constructs
both controllers; do not launch extra controller instances alongside it.
Expect all Q5.1.a-f checks to pass. For Q5.1.g it commands surge 0.3 m/s and yaw
0.2 rad/s, ignores roughly the first 50 observations, and checks both reported RMSE
values are below 0.075 over the remaining observations. It should progress to
400 observations and print results; a lack of iterations means no filtered odometry.

The localization remappings needed by this test are included in this change.
The container's existing EKF sensor-selection settings were preserved; on a
separate fresh checkout, ensure Topic 4 has configured IMU and GPS fusion.
To check that the full grader receives correct `/odometry/filtered`, use:

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
```

Do not interpret missing/bad localization as a controller arithmetic failure.
The fork does not contain the tutorial autograder package; it is supplied by the
ROS_Tutorial stack. If `autograder` is missing, verify that ROS_Tutorial is in the
workspace and rebuild/source it.

## 5. Gazebo controller-only test using ground truth

This bypasses unfinished localization without changing its files. Use fresh
terminals and a shared isolated domain, e.g. `export ROS_DOMAIN_ID=78` in each.
Source ROS and the built workspace in every terminal.

Terminal A:

```bash
ros2 launch stinger_sim sim.launch.py world:=empty.world
```

Terminal B, after Gazebo is running:

```bash
ros2 launch stinger_description spawn.launch.py
```

Terminal C:

```bash
ros2 run stinger_controller throttle_controller
```

Terminal D:

```bash
ros2 run stinger_controller velocity_controller --ros-args \
  -p use_sim_time:=true \
  -r /odometry/filtered:=/ground_truth/odometry
```

Terminal E:

```bash
ros2 topic hz /ground_truth/odometry
```

Once messages arrive, Ctrl-C and run:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

Terminal F, examine each stream in turn (Ctrl-C between commands):

```bash
ros2 topic echo /ground_truth/odometry --field twist.twist
ros2 topic echo /cmd_wrench
ros2 topic echo /stinger/thruster_port/cmd_thrust
ros2 topic echo /stinger/thruster_stbd/cmd_thrust
```

Verify forward speed approaches +0.3 m/s. In E stop the publisher and test
`{linear: {x: 0.0}, angular: {z: 0.2}}`, then the combined command
`{linear: {x: 0.3}, angular: {z: 0.2}}`. Verify yaw rate approaches +0.2 rad/s,
not a continuously growing error. For positive torque, the existing allocator
commands starboard thrust greater than port. If actual yaw has the wrong sign,
check the allocator/model conventions before trying larger PID gains.
Finally publish a zero command continuously and observe deceleration before
stopping the controllers. A zero velocity command can request braking force;
it does not necessarily produce an immediate zero wrench with PID active.

Optional plotting: `ros2 run plotjuggler plotjuggler`. Track the commanded speeds,
measured speeds, wrench, and both thruster commands. For further operating conditions, tune P first, then D to damp
oscillation, then I to remove steady error. Restart the node with new gain
parameters, e.g. append `-p Kp_surge:=20.0 -p Ki_surge:=5.0 -p Kd_surge:=1.0`.
Record RMSE and saturation for each new operating condition.

## Limits and remaining integration constraints

Defaults: force +/-60 N, torque +/-30 N m, accumulated error +/-10 on each axis,
and accepted dt from 1 microsecond through 1 second. Invalid dt or non-finite input
publishes zero, clears PID history, and primes again on the next valid sample.
The integral freezes when it would increase controller saturation and can unwind
when error reverses. All limits and gains are startup ROS parameters.

The 30 N m controller torque limit preserves the tutorial grader's deliberately
large derivative test (first yaw output 26.01 N m). It is NOT available physical
boat torque: the existing throttle allocator clips each motor to +/-30 N and uses
a 0.153 m lever arm, implying at most 9.18 N m at zero surge. Combined surge/yaw
has a smaller feasible region: abs(Fx)/2 + abs(Mz)/(2*0.153) <= 30.
This controller's anti-windup handles its own independent output limits, not
unreported downstream allocator saturation. For controller-only experiments you
can set `-p max_torque:=9.18` (the exact tutorial numeric test needs the default).
Physical lever-arm/model agreement and allocation feedback require a separate
integration check; do not assume the independent limits guarantee feasible thrust.

There is no command/odometry watchdog in this tutorial interface. Stopping a
publisher leaves the last velocity request stored, and missing odometry means no
new wrench. Use explicit zero commands for stopping these simulation tests.
