 # roboclaw-hardware-interface

Sets up the hardware interface for up to 8 Roboclaw drivers with two channels each (16 motors) with a position,
velocity, and effort joint interface making it usable for any project that uses Roboclaw drivers in any 
configuration. Since controllers depend on the specific configuration, they are left to the user to define or create.

NOTE: under heavy development. Only velocity control is supported.

## installation & usage

Connect the hardware interface to the ROS controller manager:

```c++
main()
{
  RoboclawHardwareInterface rhi;
  controller_manager::ControllerManager cm(&rhi);

  while (true)
  {
     rhi.read();
     cm.update(rhi.get_time(), rhi.get_period());
     rhi.write();
     sleep();
  }
}
```

Modify the config file according to your setup.

Launch the node and config parameters:

```
<rosparam command="load" file="$(find roboclaw-hardware-interface)/config/roboclaw_params.yaml"/>
```
