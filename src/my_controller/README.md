# `my_controller`

Here's a simple ROS2 package that demonstrates how integration test
(aka level 2 unit test) can be done using GoogleTest.

Note: The integration test used in this example is purposely kept
simple.  It makes system() calls with the following assumption:

  - Linux OS
  - pkill command

For a more portable ROS2 integration test framework, consider using catch_ros2:

  https://github.com/ngmor/catch_ros2


This ROS2 package depends on the `my_model` module.  This dependency
is specified in the package's `package.xml` file:

```
  <depend>my_model</depend>
```

Alternatively, we could also specify the dependency by creating a
`colcon.pkg` file with the content listed below.  But since this is a
ROS package, we must use `package.xml` instead of `colcon.pkg`.
Otherwise the package will not show up in the list of ROS2 packages
(ie., output of `ros2 pkg list`).

```
{
    "name": "my_controller",
    "type": "cmake",
    "dependencies" : ["my_model"]
}
```



