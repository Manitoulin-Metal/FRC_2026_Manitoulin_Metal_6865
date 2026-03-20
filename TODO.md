# AprilTag Localization for Limelight - Progress Tracker

## Task: Integrate Limelight AprilTag detections into Drive poseEstimator for robot field position estimation.

### Approved Plan Summary
- Feed filtered PoseObservations from VisionIOLimelight (MegaTag botpose) to Drive.addVisionMeasurement().
- Filter: ambiguity < 0.3, tags >=2.
- Dynamic stdDevs based on distance/tag count.

### Steps to Complete
- [x] 1. Create this TODO.md file.
- [x] 2. Edit VisionSubsystem.java: Add Drive dependency, implement pose fusion in periodic().
- [ ] 3. Edit RobotContainer.java: Pass drive to VisionSubsystem constructor.
- [x] 4. Verify/update Constants.java for SHOOTING_TAG_IDS={25,26}.
- [ ] 5. Test: Build/deploy, check AdvantageScope logs for vision-updated poses.
- [ ] 6. Tune if needed (stdDevs, filters in VisionConstants).

### Current Progress
Steps 1,2,4 complete.

**Next:** Test integration (Steps 5-6)

