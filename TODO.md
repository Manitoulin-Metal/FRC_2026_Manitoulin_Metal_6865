# Shooter PID Implementation Plan
Current status: Approved plan
## Steps:
- [x] 1. Add Shooter constants and NT tuning entries to Constants.java
- [ ] 2. Update ShooterSubsystem.java with live PID/FF tuning, periodic updatePIDIfChanged(), enhanced logging (velocity, error)
- [ ] 3. Test: Build/deploy, tune in Shuffleboard, verify RPS tracking
## Notes: Velocity PID on TalonFX Phoenix6. Initial gains from existing code.
