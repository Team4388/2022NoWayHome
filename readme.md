# Experimental
## To Do
- [x] Rewrite target tracking.
- [x] Cleanup robot map.
- [x] Cleanup robot.
- [x] Change swerve module ordering to be consistent.
- [x] Remove old-style autonomous programs.
- [x] Change HAL logging to use custom formatting on its output stream.
- [x] Use original HAL logging when custom logging is off.
- [x] Change SparkMax sendable implementation to be an extension.
- [x] Add Shuffleboard layout plan.
- [ ] Organize constants.
- [ ] Efficiently load path files.
- [ ] Change the path chooser to select autonomous commands.
- [ ] Remove unused commands.
- [ ] Rewrite controller bindings.
- [ ] Write new-style autonomous programs for lower ball counts.
- [ ] Fix all XXX comments.
- [ ] Visit TODO comments.
- [ ] Forward the limelight port so it can be accessed over a USB tether.
## Planned
- [ ] Shuffleboard replacements options:
  - [ ] Run a web server on the robot and bypass network tables.
  - [ ] Run a web server on the drive laptop that accesses network tables.
  - [ ] Write a network tables client in C++.
- [ ] Consider combining intake, serializer, and storage into a single subsystem.
- [ ] Consider combining turret, hood, and flywheels into a single subsystem.
- [ ] Move the network tables recorded path downloader into a Shuffleboard plugin.
- [ ] Finish the graph and web Shuffleboard widgets.