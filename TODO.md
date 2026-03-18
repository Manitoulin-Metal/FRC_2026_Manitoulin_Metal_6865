# Shooter Fix TODO

**Completed Steps:**
- [x] Create TODO.md with plan steps
- [x] Implement fixes in ShooterSubsystem.java (velocity closed loop, tuned PID, brake mode)
- [x] Update bindings in RobotContainer.java (sustained run at Constants.SHOOTER_VELOCITY_RPS)
- [x] Update Constants.java with SHOOTER_VELOCITY_RPS=95 RPS (~5700 RPM)
- [ ] `./gradlew deploy` and test: Hold Y on op controller (controller1), shooter should ramp to ~95 RPS. Monitor velocity in AdvantageScope/Shuffleboard.
- [ ] Tune SHOOTER_VELOCITY_RPS higher if needed (>93.3 RPS for kicker), adjust PID via TunerConstants.
