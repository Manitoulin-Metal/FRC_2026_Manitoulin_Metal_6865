package frc.robot.commands.auto;

// public class Shoot3BallsCommand extends SequentialCommandGroup {
//   private static final AprilTagFieldLayout fieldLayout =
//       AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

//   public Shoot3BallsCommand(ShooterSubsystem shooter, KickerSubsystem kicker, Drive drive) {
//     addRequirements(shooter, kicker, drive);

//     // Position to speaker tag (Limelight fused pose)
//     Pose2d tagPose = fieldLayout.getTagPose(Constants.SPEAKER_TAG_ID).get().toPose2d();
//     Transform2d offset =
//         new Transform2d(new Translation2d(-2.0, 0.0), Rotation2d.fromDegrees(180.0));
//     Pose2d targetPose =
//         new Pose2d(
//             tagPose.getTranslation().minus(offset.getTranslation()),
//             tagPose.getRotation().minus(offset.getRotation()));
//     addCommands(
//         DriveCommands.driveToPoseWithRotation(
//                 drive,
//                 targetPose,
//                 Constants.AUTO_VISION_KP_LINEAR,
//                 Constants.AUTO_VISION_KP_ANGULAR)
//             .withTimeout(4.0),
//         // Shot 1
//         Commands.parallel(
//                 Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
//                 kicker.kickerCommand(0.3))
//             .withTimeout(1.5),
//         // Shot 2
//         Commands.parallel(
//                 Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
//                 kicker.kickerCommand(0.3))
//             .withTimeout(1.5),
//         // Shot 3
//         Commands.parallel(
//                 Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
//                 kicker.kickerCommand(0.3))
//             .withTimeout(1.5),
//         // Stop
//         shooter.stopCommand().andThen(kicker.stopCommand()));
//   }
// }
