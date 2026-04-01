# Climber Limit Switch TODO
Status: In Progress

## Steps:
- [x] 1. Create/update Constants.java with Climb.LIMIT_SWITCH_CHANNEL
- [x] 2. Update ClimbSubsystem.java: add DigitalInput, isLimitSwitchPressed(), update runClimber() logic, add dashboard logging
- [ ] 3. Test: Tune DIO channel in Constants.java (change 8 to your port), deploy/deploy to roboRIO, test teleop with controller1 POV 180° (down) - should stop if switch pressed, POV 0° (up) always works. Monitor "Climb/LimitSwitchPressed" on dashboard.
- [ ] 4. Mark complete

Next step: Testing
