 # roboclaw-hardware-interface

Sets up the hardware interface for up to 8 Roboclaw drivers with two channels each (16 motors) with a position,
velocity, and effort joint interface making it usable for any project that uses Roboclaw drivers in any 
configuration. Since controllers depend on the specific configuration, they are left to the user to define or create.

## installation & usage

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

### parameters


### examples