# Shooter Fix TODO

## Steps:
- [x] Step 1: Update ShooterSubsystem.java - Add true PercentOutput open-loop control, increase default targetRps to 75.0, add ramp to runShooter.
- [x] Step 2: Update RobotContainer.java - Add high-speed shooter test binding (Operator RightBumper + Y for 75 RPS).
- [x] Step 3: Optionally bump Constants.java initial PID gains (kP=0.2, kS=0.1).

- [ ] Step 4: Build & deploy: `gradlew build deploy`
- [ ] Step 5: Test: Press Operator Y (50 RPS closed-loop), RightBumper+Y (75 RPS), Driver Y (open-loop 0.3). Monitor SmartDashboard Shooter/*.
- [ ] Step 6: Tune PID live via Shuffleboard "Tuning/Shooter/*", clear faults with LeftBumper.
- [ ] Complete: Update this file, attempt_completion.

Current: Starting Step 1.
