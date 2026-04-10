package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDMinimal;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WhipSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
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
  private final LEDMinimal led = new LEDMinimal();

  // Vision (separate cameras)
  private final Vision visionClimb;
  private final Vision visionShoot;
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

        // vision =
        // new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOLimelight("limelight", drive::getRotation),
        // new VisionIOLimelight("limelight_forward", drive::getRotation));

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
    visionClimb = new Vision(
        drive::addVisionMeasurement, new VisionIOLimelight("limelight", drive::getRotation));
    visionShoot = new Vision(
        drive::addVisionMeasurement,
        new VisionIOLimelight("limelight_forward", drive::getRotation));

    drive.setVision(visionClimb);

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

    NamedCommands.registerCommand("startIntake", intakeRoller.intakeToggleCommand());
    // NamedCommands.registerCommand("stopIntake", intakeRoller.idleCommand());

    NamedCommands.registerCommand(
        "collectFuel", intakeRoller.intakeToggleCommand().withTimeout(4.0));
    NamedCommands.registerCommand(
        "ClimbAutoDrive", ClimbCommands.autoClimbDrive(drive, fieldLayout));
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
        .back()
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
    // driver
    // .rightTrigger(0.5)
    // .whileTrue(
    // DriveCommands.driveToShootVision(
    // drive,
    // visionShoot,
    // shooter,
    // fieldLayout,
    // () -> visionEnabled,
    // () -> -driver.getLeftY(),
    // () -> -driver.getLeftX(),
    // () -> -driver.getRightX(),
    // 1.5,
    // 3.0));

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
                    intakeDeploy.deploy();
                  } else {
                    intakeDeploy.shake();
                  }
                },
                intakeDeploy));

    // Run kicker test while held.
    driver.leftTrigger(0.5).whileTrue(kicker.kickerCommand());

    // Driver climb controls: hold bumper to move, release to idle.
    driver.leftBumper().whileTrue(ClimbCommands.climbUp(climb1));
    driver.rightBumper().whileTrue(ClimbCommands.climbDown(climb1));

    // Run climb sequence: climb up, align to tag (timeout), then climb down.
    driver.y().onTrue(ClimbCommands.climbUpAlignThenDown(32, drive, visionClimb, climb1));

    // Pit calibration: position the robot where it should stop relative to the
    // climb tag,
    // then press Start to capture the current pose as the align target.
    driver.start().onTrue(DriveCommands.saveCurrentPoseAsAlignTarget(32, drive));

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

    // D-pad up moves climber up while held, respecting top lockout.
    operator.pov(0).whileTrue(ClimbCommands.climbUp(climb1));

    // D-pad down moves climber down while held, respecting bottom lockout.
    operator.pov(180).whileTrue(ClimbCommands.climbDown(climb1));

    // Run intake roller toggle.
    operator.leftTrigger(0.1).toggleOnTrue(intakeRoller.intakeToggleCommand());

    // Toggle shooter + whip on/off with each trigger press.
    operator
        .rightTrigger(0.5)
        .toggleOnTrue(ShooterCommands.shootWithWhipAndShake(shooter, whip, intakeDeploy, 75.0));

    // Run shooter at reduced speed while held.
    operator
        .y()
        .whileTrue(ShooterCommands.shootWithWhipAndShake(shooter, whip, intakeDeploy, 60.0));

    // Toggle whip command on/off.
    // operator.rightBumper().toggleOnTrue(whip.whipCommand());

    // Clear shooter sticky faults.
    operator.leftBumper().onTrue(shooter.clearFaultsCommand());
  }

  // Agitator
  // controller1.x().onTrue(intakeDeploy.deployAgitatorCommand());

  // ---------- ENABLE HOMING METHOD ----------
  public void enableHoming() {
    intakeDeploy.startHoming();
    climb1.startHoming();
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

    // LED output is handled exclusively by LEDMinimal during CANdle
    // troubleshooting.

    // Vision diagnostics inputs
    double tx = visionClimb.getTX();
    double ty = visionClimb.getTY();
    double[] offsets = new double[] { 0.0, 0.0, 0.0 };
    var climbTagPose = fieldLayout.getTagPose(Constants.CLIMB_TAG_ID);
    if (climbTagPose.isPresent()) {
      Transform2d tagToRobot = new Transform2d(climbTagPose.get().toPose2d(), drive.getPose());
      offsets[0] = tagToRobot.getX();
      offsets[1] = tagToRobot.getY();
      offsets[2] = drive
          .getPose()
          .getTranslation()
          .getDistance(climbTagPose.get().toPose2d().getTranslation());
    }

    // -------------------- SMARTDASHBOARD OUTPUTS --------------------
    SmartDashboard.putBoolean("Endgame 20s", alert20);
    SmartDashboard.putBoolean("Endgame 10s", alert10);

    SmartDashboard.putNumber("CameraToTag/measuredTX", tx);
    SmartDashboard.putNumber("CameraToTag/measuredTY", ty);
    SmartDashboard.putNumber("CameraToTag/X", offsets[0]);
    SmartDashboard.putNumber("CameraToTag/Y", offsets[1]);
    SmartDashboard.putNumber("CameraToTag/Distance", offsets[2]);

    Pose2d robotPose = drive.getPose();
    SmartDashboard.putNumber("Odometry/RobotX", robotPose.getX());
    SmartDashboard.putNumber("Odometry/RobotY", robotPose.getY());
    SmartDashboard.putNumber("Odometry/RobotRotation", robotPose.getRotation().getDegrees());

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
