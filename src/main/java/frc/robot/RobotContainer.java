package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.WhipSubsystem;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.*;

@SuppressWarnings("unused")
public class RobotContainer {

  // ============================================================
  // -------------------- SUBSYSTEMS -----------------------------
  // ============================================================

  private final Drive drive;

  private final IntakeDeploySubsystem intakeDeploy = new IntakeDeploySubsystem();
  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem(shooter);
  private final WhipSubsystem whip = new WhipSubsystem(shooter);
  private final ClimbSubsystem climb1 = new ClimbSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();

  // Vision (separate cameras)
  private final VisionSubsystem visionClimb;
  private final VisionSubsystem visionShoot;
  private boolean visionEnabled = true;

  // Toggle for robot-centric vs field-centric drive (default to field-centric)
  private boolean robotCentric = false;

  // ============================================================
  // -------------------- CONTROLLERS ----------------------------
  // ============================================================

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // ============================================================
  // -------------------- FIELD / AUTO ---------------------------
  // ============================================================

  private final Field2d field = new Field2d();

  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);

  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

  // ============================================================
  // -------------------- CONSTRUCTOR ----------------------------
  // ============================================================

  public RobotContainer() {

    // -------- Drive init --------
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        drive = new Drive(
            new GyroIOSim(),
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        break;
    }

    // -------- Vision setup --------
    visionClimb = new VisionSubsystem(new VisionIOLimelight("limelight", drive::getRotation), drive);

    visionShoot = new VisionSubsystem(new VisionIOLimelight("limelight_forward", drive::getRotation), drive);

    // -------- Default drive (now with robot-centric toggle) --------
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotCentric));

    // -------- Auto chooser --------
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // -------- Named commands --------
    NamedCommands.registerCommand("StopDrive", Commands.runOnce(drive::stop, drive));

    NamedCommands.registerCommand("startIntake", intakeRoller.intakeCommand());
    NamedCommands.registerCommand("stopIntake", intakeRoller.idleCommand());

    NamedCommands.registerCommand("collectFuel", intakeRoller.intakeCommand().withTimeout(4.0));
    NamedCommands.registerCommand("ClimbAutoDrive", ClimbCommands.autoClimbDrive(drive, fieldLayout));
    NamedCommands.registerCommand("ClimbAutoUp", ClimbCommands.autoClimbUp(climb1));
    NamedCommands.registerCommand("ClimbAutoDown", ClimbCommands.autoClimbDown(climb1));

    NamedCommands.registerCommand(
        "timedShootCommand",
        ShooterCommands.timedShoot(shooter, kicker, intakeDeploy, Constants.AUTO_SHOOT_RPS, 1.5));

    // Load autos
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
    }

    configureButtonBindings();
    // CameraServer.startAutomaticCapture(0);
  }

  // ============================================================
  // -------------------- BUTTONS --------------------------------
  // ============================================================

  private void configureButtonBindings() {
    Logger.recordOutput("Bindings/Configured", true);

    // ============================================================
    // -------------------- DRIVER BINDINGS ------------------------
    // ============================================================

    // Toggle robot-centric driving mode.
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotCentric = !robotCentric;
                  SmartDashboard.putBoolean("Drive/RobotCentric", robotCentric);
                }));

    // Toggle vision-assisted behavior.
    driver
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  visionEnabled = !visionEnabled;
                  SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
                }));

    // Stop drive outputs with X-lock.
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Zero robot heading while preserving translation.
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    // Run vision-assisted drive-to-shoot while held.
    driver
        .rightTrigger(0.5)
        .whileTrue(
            DriveCommands.driveToShootVision(
                drive,
                visionShoot,
                shooter,
                fieldLayout,
                () -> visionEnabled,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                1.5,
                3.0));

    // Log climb tag offset for calibration/debug.
    // driver
    // .a()
    // .onTrue(ClimbCommands.logClimbOffset(drive, fieldLayout, 32));

    // Toggle intake deploy shake mode on/off.
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intakeDeploy.getState() == IntakeDeploySubsystem.IntakeState.SHAKE) {
                    intakeDeploy.stow();
                  } else {
                    intakeDeploy.shake();
                  }
                },
                intakeDeploy));

    // Run kicker test while held.
    driver.leftTrigger(0.5).whileTrue(kicker.kickerCommand());

    // Run align-to-tag then climb while held.
    driver.x().whileTrue(ClimbCommands.alignAndClimb(drive, visionClimb, climb1));

    // Vision climb assist test while held (disabled).
    // driver.leftBumper().whileTrue(ClimbCommands.driveToClimbVision(drive,
    // visionClimb));

    // ============================================================
    // -------------------- OPERATOR BINDINGS ----------------------
    // ============================================================

    // Deploy intake mechanism.
    operator.a().onTrue(Commands.runOnce(intakeDeploy::deploy, intakeDeploy));

    // Stow intake mechanism.
    operator.b().onTrue(Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    // Unused x button on operator
    // operator.x().onTrue(
    // Raise climber while held.
    operator.pov(0).whileTrue(climb1.climbCommand(0.75)).onFalse(climb1.climbCommand(0));

    // Lower climber while held.
    operator.pov(180).whileTrue(climb1.climbCommand(-0.75)).onFalse(climb1.climbCommand(0));

    // Run intake roller while held.
    operator
        .leftTrigger(0.1)
        .whileTrue(intakeRoller.intakeCommand())
        .onFalse(intakeRoller.idleCommand());

    // Toggle shooter + whip on/off with each trigger press.
    operator
        .rightTrigger(0.5)
        .toggleOnTrue(ShooterCommands.shootWithWhip(shooter, whip, intakeDeploy, 75.0));

    // Run shooter at reduced speed while held.
    operator
        .y()
        .whileTrue(ShooterCommands.runShooterAtRps(shooter, intakeDeploy, 60.0));

    // Toggle whip command on/off.
    operator.rightBumper().toggleOnTrue(whip.whipCommand());

    // Clear shooter sticky faults.
    operator.leftBumper().onTrue(shooter.clearFaultsCommand());
  }

  // Agitator
  // controller1.x().onTrue(intakeDeploy.deployAgitatorCommand());

  // ---------- ENABLE HOMING METHOD ----------
  public void enableHoming() {
    intakeDeploy.startHoming();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // ============================================================
  // -------------------- PERIODIC ------------------------------
  // ============================================================

  public void periodic() {
    // Snapshot inputs once so outputs are consistent between sinks.
    double driverLeftY = driver.getLeftY();
    double driverLeftX = driver.getLeftX();
    double driverRightX = driver.getRightX();
    double driverRT = driver.getRightTriggerAxis();
    double driverLT = driver.getLeftTriggerAxis();
    double operatorLeftY = operator.getLeftY();
    double operatorLT = operator.getLeftTriggerAxis();
    double operatorRT = operator.getRightTriggerAxis();

    double matchTime = DriverStation.getMatchTime();

    boolean alert20 = matchTime > 0 && matchTime <= endgameAlert1.get();
    boolean alert10 = matchTime > 0 && matchTime <= endgameAlert2.get();

    double rumble = (alert20 || alert10) ? 0.5 : 0.0;

    driver.getHID().setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kLeftRumble, rumble);
    driver.getHID().setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kRightRumble, rumble);

    // LED + gyro alerts
    if (drive.isGyroDisconnected()) {
      led.gyroDisconnectedAlert();
    }

    // Vision diagnostics inputs
    double tx = visionClimb.getTX();
    double ty = visionClimb.getTY();
    double[] offsets = visionClimb.getCameraToTagOffset(fieldLayout, Constants.CLIMB_TAG_ID, tx, ty);

    // -------------------- SMARTDASHBOARD OUTPUTS --------------------
    SmartDashboard.putBoolean("Endgame 20s", alert20);
    SmartDashboard.putBoolean("Endgame 10s", alert10);

    SmartDashboard.putNumber("CameraToTag/measuredTX", tx);
    SmartDashboard.putNumber("CameraToTag/measuredTY", ty);
    SmartDashboard.putNumber("CameraToTag/X", offsets[0]);
    SmartDashboard.putNumber("CameraToTag/Y", offsets[1]);
    SmartDashboard.putNumber("CameraToTag/Distance", offsets[2]);

    SmartDashboard.putNumber("Driver/LeftY", driverLeftY);
    SmartDashboard.putNumber("Operator/LeftY", operatorLeftY);
    SmartDashboard.putBoolean("Vision Enabled", visionEnabled);

    // -------------------- LOGGER OUTPUTS --------------------
    Logger.recordOutput("Controls/DriverLeftY", driverLeftY);
    Logger.recordOutput("Controls/DriverLeftX", driverLeftX);
    Logger.recordOutput("Controls/DriverRightX", driverRightX);
    Logger.recordOutput("Controls/DriverRT", driverRT);
    Logger.recordOutput("Controls/DriverLT", driverLT);
    Logger.recordOutput("Controls/OperatorLT", operatorLT);
    Logger.recordOutput("Controls/OperatorRT", operatorRT);
    Logger.recordOutput("Match/Endgame20", alert20);
    Logger.recordOutput("Match/Endgame10", alert10);
  }
}
