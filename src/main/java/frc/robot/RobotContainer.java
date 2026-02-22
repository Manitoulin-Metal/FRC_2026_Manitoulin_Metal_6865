package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

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

    // This automatically loads ALL autos from the deploy folder
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
    }

    // Configure buttons
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Hold left trigger to drive to AprilTag 26
    controller
        .leftTrigger(0.5)
        .whileTrue(
            DriveCommands.driveToShoot(
                drive, fieldLayout, 1.5, 3.0, !edu.wpi.first.wpilibj.RobotBase.isSimulation()));

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
    return DriveCommands.driveToShoot(
        drive, fieldLayout, 1.5, 3.0, !edu.wpi.first.wpilibj.RobotBase.isSimulation());
  }

  // ---------------- Periodic method for endgame alerts ----------------
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();

    boolean alert20 = matchTime > 0 && matchTime <= endgameAlert1.get();
    boolean alert10 = matchTime > 0 && matchTime <= endgameAlert2.get();

    // SmartDashboard display
    SmartDashboard.putBoolean("Endgame 20s", alert20);
    SmartDashboard.putBoolean("Endgame 10s", alert10);

    // AdvantageScope logging
    Logger.recordOutput("Match/Endgame20", alert20);
    Logger.recordOutput("Match/Endgame10", alert10);

    // ------------- Controller rumble ----------------
    double rumbleIntensity = 0.5; // 0.0 to 1.0
    if (alert20 || alert10) {
      controller.setRumble(
          edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, rumbleIntensity);
      controller.setRumble(
          edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, rumbleIntensity);
    } else {
      controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble, 0.0);
      controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble, 0.0);
    }

    // Optional: flash SmartDashboard color for 10s alert (requires Shuffleboard,
    // pseudo code)
    if (alert10) {
      SmartDashboard.putString("Endgame Warning Color", "RED");
    } else if (alert20) {
      SmartDashboard.putString("Endgame Warning Color", "YELLOW");
    } else {
      SmartDashboard.putString("Endgame Warning Color", "NONE");
    }
  }
}
