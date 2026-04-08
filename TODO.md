# Replace LimelightHelpers in DriveCommands.java with WPILib PIDController

## Approved Plan Steps

### 1. ✅ Create TODO.md (Current step - completed)

### 2. ✅ Create PIDControllers in DriveCommands.java
### 3. ✅ Refactor alignToTag method
### 4. ✅ Update import statements

### 5. Update callers in RobotContainer.java
- Find alignToTag calls and pass VisionSubsystem instance

### 6. Add constants to Constants.java (optional)
- PID gains: ALIGN_STRAFE_P, ALIGN_DISTANCE_P, ALIGN_ROT_P

### 7. Test and tune
- Verify tag alignment works identically or better
- Tune PID gains via Shuffleboard
- ✅ attempt_completion

**Status:** Ready for implementation
