package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.Shoot3BallsCommand;
import frc.robot.commands.auto.SimpleDriveAndSpinAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.intakedeploy.IntakeDeploySubsystem;
import frc.robot.subsystems.intake.intakeroller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@SuppressWarnings("unused")
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final IntakeDeploySubsystem intakeDeploy = new IntakeDeploySubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem(shooter);
  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final ClimbSubsystem climb1 = new ClimbSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();

  private VisionSubsystem vision;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller1 = new CommandXboxController(1);

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
                new GyroIOSim(),
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

    // Instantiate vision subsystem
    vision = new VisionSubsystem(new VisionIOLimelight("limelight", drive::getRotation), drive);

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
        "Vision Shoot3 Positioned", new Shoot3BallsCommand(shooter, kicker, drive));
    autoChooser.addOption(
        "Drive to Climb (coordinates)",
        DriveCommands.driveToClimb(
            drive, fieldLayout, 1.5, 3.0, !edu.wpi.first.wpilibj.RobotBase.isSimulation()));

    // This automatically loads ALL autos from the deploy folder
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
    }

    // Register NamedCommands FIRST
    NamedCommands.registerCommand("StopDrive", Commands.runOnce(drive::stop, drive));
    NamedCommands.registerCommand("startIntake", intakeRoller.intakeCommand());
    NamedCommands.registerCommand("stopIntake", intakeRoller.idleCommand());
    NamedCommands.registerCommand("collectFuel", intakeRoller.intakeCommand().withTimeout(4.0));
    NamedCommands.registerCommand("startClimb", climb1.ClimbCommand(0.5));
    NamedCommands.registerCommand("shoot3Balls", new Shoot3BallsCommand(shooter, kicker, drive));

    // Configure buttons
    configureButtonBindings();

    CameraServer.startAutomaticCapture(0);
  }

  private void configureButtonBindings() {

    // Hold left trigger to drive to AprilTag 26
    // (Driver Controller)
    /* controller
    .leftTrigger(0.5)
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
                  !edu.wpi.first.wpilibj.RobotBase.isSimulation());
            },
            Set.of(drive) // <-- required subsystem set
            ));
            */

    // (Driver Controller)
    // Hold right trigger to drive to climb position (Tag 31)
    controller
        .rightTrigger(0.5)
        .whileTrue(
            DriveCommands.driveToClimb(
                drive, fieldLayout, 1.5, 3.0, !edu.wpi.first.wpilibj.RobotBase.isSimulation()));

    // Deploy intake to PID setpoint (Operator Controller)
    controller1.a().onTrue(intakeDeploy.deployCommand());

    // Stow intake to PID setpoint (Operator Controller)
    controller1.b().onTrue(intakeDeploy.stowCommand());

    // When Right Trigger pressed, run intake rollers; when released, stop rollers
    // (For Operator Controller)
    // controller1
    //     .rightTrigger(0.1)
    //     .whileTrue(intakeRoller.intakeCommand())
    //     .onFalse(intakeRoller.idleCommand());

    // Switch to X pattern when X button pressed
    // (Driver Controller)
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // When Button Y held, Vision-distance adjusted auto-shoot (Operator Controller)
    controller1
        .y()
        .whileTrue(Commands.run(() -> shooter.runVisionShooter(vision), vision, shooter))
        .onFalse(shooter.stopCommand());

    // Operator LeftBumper: Clear shooter sticky faults
    controller1.leftBumper().onTrue(shooter.clearFaultsCommand());

    // Operator RightBumper + Y: High speed shooter test (75 RPS)
    // controller1
    //     .rightBumper()
    //     .and(controller1.y())
    //     .whileTrue(Commands.run(() -> shooter.runShooter(75.0), shooter))
    //     .onFalse(shooter.stopCommand());

    // Deploy agitator on X (fast shake then stow)
    controller1.x().onTrue(intakeDeploy.deployAgitatorCommand());

    // Reset gyro to 0° when B pressed
    // (Driver Controller)
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    // When Left Bumper held, Climber moves downward (climbs)
    // (Driver Controller)
    controller.leftBumper().whileTrue(climb1.ClimbCommand(0.5));

    // When Left Bumper released, Climber stops
    // (Driver Controller)
    controller.leftBumper().onFalse(climb1.ClimbCommand(0));

    // When Right Bumper held, Climber Raises
    // (Driver Controller)
    controller.rightBumper().onTrue(climb1.ClimbCommand(-0.5));

    // Temporary: Driver LT runs kicker at -0.3 to test motor
    controller.leftTrigger(0.5).whileTrue(kicker.kickerCommand(0.3));

    // Driver Y: Test shooter open-loop 30% duty (hardware test, previously unused)
    // controller
    //     .y()
    //     .whileTrue(Commands.run(() -> shooter.runOpenLoop(0.2), shooter))
    //     .onFalse(shooter.stopCommand());

    // -------- New controls as of 3/20 based on driver input ----------

    // (Operator) Left Trigger: Spin Intake Rollers
    controller1
        .leftTrigger(0.1)
        .whileTrue(intakeRoller.intakeCommand())
        .onFalse(intakeRoller.idleCommand());

    // (Operator) Right Trigger: Shooter + Kicker
    controller1
        .rightTrigger(0.1)
        .whileTrue(Commands.run(() -> shooter.runShooter(75.0), shooter))
        .onFalse(shooter.stopCommand());
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

    // ------------- Gyro disconnected alert ----------------
    if (drive.isGyroDisconnected()) {
      led.gyroDisconnectedAlert();
    }

    // ------------- Operator Controller Diagnostics ----------------
    // Driver controller (port 0) for comparison
    SmartDashboard.putNumber("Driver/LeftY", controller.getLeftY());
    SmartDashboard.putNumber("Driver/RightX", controller.getRightX());

    // Operator controller (port 1) diagnostics
    SmartDashboard.putNumber("Operator/LeftY", controller1.getLeftY());
    SmartDashboard.putNumber("Operator/LeftX", controller1.getLeftX());
    SmartDashboard.putNumber("Operator/RightX", controller1.getRightX());
    SmartDashboard.putNumber("Operator/LeftTrigger", controller1.getLeftTriggerAxis());
    SmartDashboard.putNumber("Operator/RightTrigger", controller1.getRightTriggerAxis());
    SmartDashboard.putBoolean("Operator/A", controller1.a().getAsBoolean());
    SmartDashboard.putBoolean("Operator/B", controller1.b().getAsBoolean());
    SmartDashboard.putBoolean("Operator/Y", controller1.y().getAsBoolean());
    SmartDashboard.putBoolean("Operator/RightBumper", controller1.rightBumper().getAsBoolean());
    SmartDashboard.putBoolean(
        "Operator/Y+RB",
        controller1.y().getAsBoolean() && controller1.rightBumper().getAsBoolean());
  }
}
