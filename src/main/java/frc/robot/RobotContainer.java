package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final Drive drive;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Field2d field = new Field2d();

  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIOSim(),
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    SmartDashboard.putData("Field", field);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // LEFT TRIGGER → Drive to Tag 2 (if confirmed)
    controller.leftTrigger(0.5).whileTrue(driveToTag2IfVisible());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // ==============================================================
  // MAIN VISION DRIVE COMMAND
  // ==============================================================

  private Command driveToTag2IfVisible() {

    return new RunCommand(
        () -> {

          // ------------------------------------------------
          // 1. CHECK LIMELIGHT FOR TAG 2
          // ------------------------------------------------

          boolean hasTarget = LimelightHelpers.getTV("limelight");
          double tagID = LimelightHelpers.getFiducialID("limelight");

          if (!hasTarget || tagID != 2) {
            drive.stop();
            return;
          }

          // ------------------------------------------------
          // 2. GET LIMELIGHT ROBOT POSE (MegaTag2 recommended)
          // ------------------------------------------------

          LimelightHelpers.PoseEstimate estimate =
              LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

          if (estimate == null || estimate.pose == null) {
            drive.stop();
            return;
          }

          Pose2d visionPose = estimate.pose;

          // ------------------------------------------------
          // 3. BLEND INTO POSE ESTIMATOR
          // ------------------------------------------------

          Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
          // trust X/Y moderately, ignore rotation (LL rotation can be noisy)

          drive.addVisionMeasurement(visionPose, estimate.timestampSeconds, visionStdDevs);

          // ------------------------------------------------
          // 4. GET TARGET TAG POSE FROM FIELD LAYOUT
          // ------------------------------------------------

          Optional<edu.wpi.first.math.geometry.Pose3d> tagOptional = fieldLayout.getTagPose(2);

          if (tagOptional.isEmpty()) {
            drive.stop();
            return;
          }

          Pose2d tagPose = tagOptional.get().toPose2d();

          // ------------------------------------------------
          // 5. AUTO-FLIP FOR ALLIANCE
          // ------------------------------------------------

          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

            tagPose =
                new Pose2d(
                    fieldLayout.getFieldLength() - tagPose.getX(),
                    tagPose.getY(),
                    tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
          }

          // ------------------------------------------------
          // 6. TARGET = 2 FEET IN FRONT OF TAG
          // ------------------------------------------------

          double offsetMeters = 0.6096;

          Pose2d targetPose =
              tagPose.transformBy(
                  new Transform2d(new Translation2d(offsetMeters, 0), Rotation2d.kZero));

          field.setRobotPose(drive.getPose());
          field.getObject("Target").setPose(targetPose);

          // ------------------------------------------------
          // 7. HOLONOMIC CONTROLLER
          // ------------------------------------------------

          PIDController xController = new PIDController(2.5, 0, 0);
          PIDController yController = new PIDController(2.5, 0, 0);

          ProfiledPIDController thetaController =
              new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(3, 3));

          thetaController.enableContinuousInput(-Math.PI, Math.PI);

          HolonomicDriveController controller =
              new HolonomicDriveController(xController, yController, thetaController);

          var speeds =
              controller.calculate(drive.getPose(), targetPose, 0, targetPose.getRotation());

          drive.runVelocity(speeds);
        },
        drive);
  }
}
