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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
// import frc.robot.subsystems.LEDMinimal;
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
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  private final IntakeDeploySubsystem intakeDeploy = new IntakeDeploySubsystem();
  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem(shooter);
  private final WhipSubsystem whip = new WhipSubsystem(shooter);
  private final ClimbSubsystem climb1 = new ClimbSubsystem();
  // private final LEDMinimal led = new LEDMinimal();

  // Vision (separate cameras)
  private Vision vision; // The main Vision Class
  private boolean visionEnabled =
      true; // This is false because Cameras are unplugged, but don't change

  // Toggle for robot-centric vs field-centric drive (default to field-centric)
  private boolean robotCentric = false;

  // ============================================================
  // -------------------- CONTROLLERS ----------------------------
  // ============================================================

  private final CommandXboxController driver = new CommandXboxController(0); // Driver Controller

  private final CommandXboxController operator =
      new CommandXboxController(1); // Operator Controller

  // ============================================================
  // -------------------- FIELD / AUTO ---------------------------
  // ============================================================

  private final Field2d field = new Field2d();

  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);

  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

  // ============================================================
  // -------------------- CONSTRUCTOR ----------------------------
  // ============================================================

  public RobotContainer() {

    // -------- Drive init --------
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
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

    // -------- Vision setup --------
    vision =
        new Vision(
            drive::addVisionMeasurement, new VisionIOLimelight("limelight", drive::getRotation));
    drive.setVision(vision);

    // -------- Default drive (now with robot-centric toggle) --------
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotCentric));

    // -------- Auto chooser --------
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // -------- Named commands --------
    NamedCommands.registerCommand("StopDrive", Commands.runOnce(drive::stop, drive));

    NamedCommands.registerCommand("startIntake", intakeRoller.intakeToggleCommand());
    // NamedCommands.registerCommand("stopIntake", intakeRoller.idleCommand());

    NamedCommands.registerCommand(
        "collectFuel", intakeRoller.intakeToggleCommand().withTimeout(3.0));
    NamedCommands.registerCommand(
        "ClimbAutoDrive", ClimbCommands.autoClimbDrive(drive, fieldLayout));

    // Allows the Climber to go Up in an AutoCommand
    NamedCommands.registerCommand("ClimbAutoUp", ClimbCommands.autoClimbUp(climb1));

    // Allows the Climber to go Down in an AutoCommand
    NamedCommands.registerCommand("ClimbAutoDown", ClimbCommands.autoClimbDown(climb1));

    // New (UNTESTED) - Should Fix The Shooter not Shooting Problem
    NamedCommands.registerCommand(
        "timedShootCommand",
        ShooterCommands.shootWithWhipAndShake(shooter, whip, intakeDeploy, 75.0)
            .withTimeout(7.5)
            .finallyDo(
                interrupted -> {
                  shooter.stopShooter();
                  intakeDeploy.deploy();
                }));

    // Allows the IntakeDeploy to be Deployed in an AutoCommand
    NamedCommands.registerCommand(
        "intakeDeploy", Commands.runOnce(intakeDeploy::deploy, intakeDeploy));

    // Allows the IntakeDeploy to be Stowed in an AutoCommand
    NamedCommands.registerCommand("intakeStow", Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    // Allows the IntakeRoller to stop running in an AutoCommand
    NamedCommands.registerCommand("StopCollectingFuel", intakeRoller.StopIntakeCommand());

    // This loads the autos for the technician to choose an auto on SmartDashboard/Shuffleboard
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
    }

    // ========== NEW AS OF 9:46AM 4/12/2026 ========== \\
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    //  \\ ================================================= //

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

    drive.setDefaultCommand(
        DriveCommands.JoystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    // Toggle robot-centric driving mode. Press Again To Disable Robot-Centric
    // Driving Mode And
    // Switch To Field-Centric Driving Mode
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotCentric = !robotCentric;
                  SmartDashboard.putBoolean("Drive/RobotCentric", robotCentric);
                }));

    // Toggle vision-assisted behavior. Press Again To Turn Off Vision.
    driver
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  visionEnabled = !visionEnabled;
                  SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
                }));

    // Stop drive outputs with X-lock (All Wheels Turn Inwards For A Full Stop).
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Zero robot heading while preserving translation.
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

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

    // Run Kicker Test While Holding Left Trigger On Driver Controller.
    driver.leftTrigger(0.5).whileTrue(kicker.kickerCommand());

    // Run Align-To-Tag Then Climb While Holding Y Button On Driver Controller.
    driver.y().whileTrue(DriveCommands.alignToTag(32, drive, vision));

    // ============================================================
    // -------------------- OPERATOR BINDINGS ----------------------
    // ============================================================

    // Deploy Intake Deploy Mechanism Toggled When A Button Pressed On Operator
    // Controller.
    operator.a().onTrue(Commands.runOnce(intakeDeploy::deploy, intakeDeploy));

    // Stow Intake Deploy Mechanism Toggled When B Button Pressed On Operator
    // Controller.
    operator.b().onTrue(Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    // Unused x button on operator
    // operator.x().onTrue(

    // Lowers The Climber While Holding D-Pad Up.
    operator.pov(0).whileTrue(climb1.climbCommand(0.75)).onFalse(climb1.climbCommand(0));

    // Raises The Climber While Holding D-Pad Down.
    operator.pov(180).whileTrue(climb1.climbCommand(-0.75)).onFalse(climb1.climbCommand(0));

    // Run Intake Roller With A Toggle When Left Trigger Pressed On Operator
    // Controller.
    operator.leftTrigger(0.1).toggleOnTrue(intakeRoller.intakeToggleCommand());

    // Toggles Shooter + Whip On/Off With Each Trigger Press.
    operator
        .rightTrigger(0.5)
        .toggleOnTrue(ShooterCommands.shootWithWhipAndShake(shooter, whip, intakeDeploy, 75.0));

    // Run Shooter At Reduced Speed While Operator Holds Y Button.
    operator
        .y()
        .whileTrue(ShooterCommands.shootWithWhipAndShake(shooter, whip, intakeDeploy, 60.0));

    // Toggle Whip Command On/Off (Solo Command).
    // operator.rightBumper().toggleOnTrue(whip.whipCommand());

    // When LeftBumper Toggled, The intake reverses rotation direction
    operator.leftBumper().toggleOnTrue(intakeRoller.ReverseCommand());
  }

  // Agitator
  // controller1.x().onTrue(intakeDeploy.deployAgitatorCommand());

  // ---------- ENABLE HOMING METHOD ----------
  public void enableHoming() {
    intakeDeploy.startHoming();
    CommandScheduler.getInstance().schedule(climb1.homeCommand());
  }

  // Allows the Shuffleboard/SmartDashBoard to select an auto
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

    // MOVE THIS VISION DIAGNOSTIC CODE TO VISION SUBSYSTEM PERIODIC

    // double tx = vision.getTX();
    // double ty = vision.getTY();
    // double[] offsets = new double[] {0.0, 0.0, 0.0};
    // var climbTagPose = fieldLayout.getTagPose(Constants.CLIMB_TAG_ID);
    // if (climbTagPose.isPresent()) {
    //   Transform2d tagToRobot = new Transform2d(climbTagPose.get().toPose2d(), drive.getPose());
    //   offsets[0] = tagToRobot.getX();
    //   offsets[1] = tagToRobot.getY();
    //   offsets[2] =
    //       drive
    //           .getPose()
    //           .getTranslation()
    //           .getDistance(climbTagPose.get().toPose2d().getTranslation());
    // }

    

    // -------------------- SMARTDASHBOARD OUTPUTS --------------------

    // Uncomment for runing and debugging - commented to avoid loop overun

    // SmartDashboard.putBoolean("Endgame 20s", alert20);
    // SmartDashboard.putBoolean("Endgame 10s", alert10);
    // SmartDashboard.putNumber("CameraToTag/measuredTX", tx);
    // SmartDashboard.putNumber("CameraToTag/measuredTY", ty);
    // SmartDashboard.putNumber("CameraToTag/X", offsets[0]);
    // SmartDashboard.putNumber("CameraToTag/Y", offsets[1]);
    // SmartDashboard.putNumber("CameraToTag/Distance", offsets[2]);
    // SmartDashboard.putNumber("Driver/LeftY", driverLeftY);
    // SmartDashboard.putNumber("Operator/LeftY", operatorLeftY);
    // SmartDashboard.putBoolean("Vision Enabled", visionEnabled);

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
