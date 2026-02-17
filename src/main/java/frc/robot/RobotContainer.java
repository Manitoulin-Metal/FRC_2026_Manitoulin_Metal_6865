package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.Collections;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.SimpleDriveAndSpinAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.vision.VisionIO.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@SuppressWarnings("unused")
public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);

  // Field display
  private final Field2d field = new Field2d();

  // AprilTag layout 2026
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Instantiate Drive depending on mode
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
                new GyroIOPigeon2(),
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default: // REPLAY
        drive =
            new Drive(
                new GyroIO() {
                  @Override
                  public void updateInputs(GyroIOInputs inputs) {}
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {}
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {}
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {}
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {}
                });
        break;
    }

    // Configure default drive command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addOption("Simple Drive + Spin", new SimpleDriveAndSpinAuto(drive));
    autoChooser.addOption("Drive to Shoot", driveToShoot());
    autoChooser.addOption(
        "Drive to Climb (coordinates)",
        DriveCommands.driveToClimb(
            drive, fieldLayout, 1.5, 3.0, !edu.wpi.first.wpilibj.RobotBase.isSimulation()));

    // Configure buttons
    configureButtonBindings();
  }

  private void configureButtonBindings() {

   // Hold left trigger to drive to AprilTag 26
controller.leftTrigger(0.5)
    .whileTrue(
        Commands.defer(
            () -> {
                // Get Pose2d for Tag 26
                var tagOptional = fieldLayout.getTagPose(26);
                if (tagOptional.isEmpty()) {
                    return Commands.none(); // Do nothing if tag not found
                }
                Pose2d tag26Pose = tagOptional.get().toPose2d();

                // Return the driveToShoot command
                return DriveCommands.driveToShoot(
                    drive,
                    tag26Pose,
                    1.5, // kP linear
                    3.0, // kP rotation
                    fieldLayout,
                    !edu.wpi.first.wpilibj.RobotBase.isSimulation()
                );
            },
            Set.of(drive) // <-- required subsystem set
        )
    );


    // Hold right trigger to drive to climb position (Tag 31)
    controller
        .rightTrigger(0.5)
        .whileTrue(
            DriveCommands.driveToClimb(
                drive, fieldLayout, 1.5, 3.0, !edu.wpi.first.wpilibj.RobotBase.isSimulation()));

    // Lock to 0° when A button held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
  }

  /** Returns the autonomous command selected on dashboard */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Drive to AprilTag 26 using DriveCommands.driveToPose */
  private Command driveToShoot() {

    var tagOptional = fieldLayout.getTagPose(26);
    if (tagOptional.isEmpty()) {
      return new InstantCommand(); // Do nothing if tag not found
    }

    Pose2d tagPose = tagOptional.get().toPose2d();
    Transform2d offset = new Transform2d(new Translation2d(0.9, 0.0), Rotation2d.kZero);
    Pose2d targetPose = tagPose.transformBy(offset);

    // Use your DriveCommands.driveToPose
    return DriveCommands.driveToShoot(
        drive,
        targetPose,
        1.5, // kP linear
        3.0, // kP rotation
        fieldLayout,
        !edu.wpi.first.wpilibj.RobotBase.isSimulation());
  }
}
