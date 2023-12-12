# Seeker Swarm

[![codecov](https://codecov.io/gh/TommyChangUMD/ENPM808X-final-project-boilerplate/branch/main/graph/badge.svg?token=KRAHD3BZP7)](https://codecov.io/gh/TommyChangUMD/ENPM808X-final-project-boilerplate)

![CICD Workflow status](https://github.com/TommyChangUMD/ENPM808X-final-project-boilerplate/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

This repo provides a template for setting up:

  - GitHub CI
    - "main" branch runs in a ROS 2 Humble container
  - Codecov badges
  - Colcon workspace structure
  - C++ library that depends on other system libraries such as OpenCV and rclcpp.
    - The library is *self-contained*
    - In real life, we download source code of third-party modules all
      the time and often just stick the modules as-is into our colcon
      workspace.
  - ROS 2 package that depends on a C++ library built in the same colcon workspace
  - Establishing package dependency within the colcon workspace.
    - ie. the ROS 2 package will not be built before all of its dependent C++ libraries are built first
  - Multiple subscriptions within a ROS2 node all listening to the same topic.
    - Only one callback function is needed.
    - More efficient than to have N callback functions.
    - More efficient than to have N ROS nodes.
  - Unit test and integration test.
  - Doxygen setup
  - ROS2 launch file
  - Bash scripts that can be invoked by the "ros2 run ..." command
  
## How to generate package dependency graph

``` bash
colcon graph --dot | dot -Tpng -o depGraph.png
open depGraph.png
```
[<img src=screenshots/depGraph.png
    width="20%" 
    style="display: block; margin: 0 auto"
    />](screenshots/depGraph.png)



## How to build and run demo

```bash
rm -rf build/ install/
colcon build 
source install/setup.bash
ros2 launch my_controller run_demo.launch.py
```

## How to build for tests (unit test and integration test)

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Test coverage report for `my_controller`:

``` bash
ros2 run my_controller generate_coverage_report.bash
open build/my_controller/test_coverage/index.html
```

### Test coverage report for `my_model`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select my_model \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/my_model/test_coverage/index.html
```

### combined test coverage report

``` bash
./do-tests.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```

## How to use GitHub CI to upload coverage report to Codecov

### First, sign up Codecov with you GitHub account.

  https://about.codecov.io/sign-up/

### Then, follow the similar instruction provided in the cpp-boilerplate-v2 repo

  https://github.com/TommyChangUMD/cpp-boilerplate-v2
