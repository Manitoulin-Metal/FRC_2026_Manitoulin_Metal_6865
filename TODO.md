# FRC Auto Limelight Bump Correction Task ✓

## Information Gathered
[Same as above]

## Implementation Steps
- [x] 0. Create TODO.md with plan ✓
- [x] 1. Add constants to Constants.java (AUTO_BUMP_ERROR_METERS=0.5, AUTO_BUMP_YAW_DEG=10.0) ✓

Progress: Step 2 next
- [ ] 2. Modify VisionSubsystem.java: Store/track latest valid pose, expose getLatestValidPose()
- [ ] 3. Modify Drive.java: In periodic(), if autonomous && vision.hasValidPose() && error > threshold, setPose(vision.getLatestValidPose())
- [ ] 4. RobotContainer.java: Register NamedCommands.registerCommand("visionReset", Commands.runOnce(() -> drive.setPose(vision.getLatestValidPose()), drive, vision))
- [ ] 5. Update TODO.md: Mark complete, add test instructions
- [ ] 6. Test: Manual verification, suggest sim bump test

Progress: Starting Step 1

