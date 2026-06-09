package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.*;

@SuppressWarnings("unused")
public class RobotContainer {

  // ============================================================
  // DRIVETRAIN
  // ============================================================
  private final Drive drive;

  private boolean robotCentric = false;

  // ============================================================
  // CONTROLLERS
  // ============================================================
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // ============================================================
  // SUBSYSTEMS
  // ============================================================
  private final LEDSubsystem led = new LEDSubsystem();
  private final IntakeDeploySubsystem intakeDeploy = new IntakeDeploySubsystem();
  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem(led);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem(shooter);
  private final WhipSubsystem whip = new WhipSubsystem(shooter);
  private final ClimbSubsystem climb = new ClimbSubsystem(led);

  // ============================================================
  // VISION
  // ============================================================
  private final Vision vision;
  private boolean visionEnabled = true;

  // ============================================================
  // FIELD / AUTO
  // ============================================================
  private final Field2d field = new Field2d();

  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert 20", 20.0);

  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert 10", 10.0);

  // ============================================================
  // CONSTRUCTOR
  // ============================================================
  public RobotContainer() {

    // ---------------- DRIVE INIT ----------------
    drive = createDrive();

    // ---------------- VISION INIT (CLEAN) ----------------

    vision =
        new Vision(
            drive::addVisionMeasurement,
            () -> drive.getPose(),
            drive,
            new VisionIOLimelight(VisionConstants.rearCameraName, drive::getRotation),
            new VisionIOLimelight(VisionConstants.frontCameraName, drive::getRotation));

    drive.setVision(vision);

    // ---------------- DEFAULT DRIVE ----------------
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotCentric));

    // ---------------- NAMED COMMANDS ----------------
    registerNamedCommands();
    // ---------------- AUTO ----------------

    autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());

    configureBindings();
  }

  // ============================================================
  // DRIVE FACTORY
  // ============================================================
  private Drive createDrive() {
    return switch (Constants.currentMode) {
      case REAL -> new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

      case SIM -> new Drive(
          new GyroIOSim(),
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));

      default -> new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    };
  }

  // ============================================================
  // NAMED COMMANDS
  // ============================================================
  private void registerNamedCommands() {

    NamedCommands.registerCommand("StopDrive", Commands.runOnce(drive::stop, drive));

    NamedCommands.registerCommand(
        "IntakeDeploy", Commands.runOnce(intakeDeploy::deploy, intakeDeploy));

    NamedCommands.registerCommand("intakeStow", Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    // NamedCommands.registerCommand("ClimbAutoUp",
    // ClimbCommands.climbUp(climb).withTimeout(2.0));

    // wraps command with a check to see if
    // climber is homed before raising hook
    // NamedCommands.registerCommand(
    //         "ClimbAutoUp",
    //
    // ClimbCommands.waitForHome(climb).andThen(ClimbCommands.hookUp(climb).withTimeout(3.0)));

    NamedCommands.registerCommand(
        "ClimbAutoUp",
        Constants.currentMode == Constants.Mode.SIM
            ? ClimbCommands.hookUp(climb).withTimeout(3.0)
            : ClimbCommands.waitForHome(climb)
                .andThen(ClimbCommands.hookUp(climb).withTimeout(3.0)));

    NamedCommands.registerCommand(
        "DockToClimb", DriveCommands.dockToClimb(drive, vision).withTimeout(4.0));

    NamedCommands.registerCommand("ClimbAutoDown", ClimbCommands.climbDown(climb).withTimeout(1.0));
    NamedCommands.registerCommand(
        "wave", Commands.runOnce(intakeDeploy::shake, intakeDeploy).withTimeout(2.0));
    NamedCommands.registerCommand(
        "Shoot",
        ShooterCommands.shootWithWhipAndShake(shooter, whip, kicker, intakeDeploy, led, 48.0)
            .withTimeout(5.0));
  }

  private Command visionTestCommand() {
    return Commands.runOnce(
        () -> {
          Pose2d tagPose = vision.getBestEstimatedPose().orElse(null);

          if (tagPose == null) {
            System.out.println("No vision pose detected");
            return;
          }

          // Tag 32 assumed already filtered by Vision system OR you verify externally
          Translation2d offset =
              new Translation2d(1.0, 0.25); // 1m forward, 0.25m left (field frame)

          Pose2d targetPose =
              new Pose2d(
                  tagPose.getX() + offset.getX(),
                  tagPose.getY() + offset.getY(),
                  tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));

          System.out.println("Vision test target: " + targetPose);

          // Use PathPlanner built-in pathfind (cleanest way in your stack)
          CommandScheduler.getInstance()
              .schedule(
                  AutoBuilder.pathfindToPose(
                      targetPose,
                      new PathConstraints(
                          2.0, // max speed m/s
                          2.0, // accel
                          Math.PI, // max angular speed
                          Math.PI // angular accel
                          ),
                      0.0));
        },
        drive);
  }

  // ============================================================
  // BINDINGS
  // ============================================================
  private void configureBindings() {

    // ---------------- DRIVER ----------------
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotCentric = !robotCentric;
                  SmartDashboard.putBoolean("Drive/RobotCentric", robotCentric);
                }));

    driver.y().whileTrue(DriveCommands.dockToClimb(drive, vision));

    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    // ---------------- OPERATOR ----------------
    operator.a().onTrue(Commands.runOnce(intakeDeploy::deploy, intakeDeploy));
    operator.b().onTrue(Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    operator.pov(0).whileTrue(ClimbCommands.hookUp(climb)).onFalse(ClimbCommands.stop(climb));

    operator.pov(180).whileTrue(ClimbCommands.climbDown(climb)).onFalse(ClimbCommands.stop(climb));

    operator.leftTrigger(0.1).toggleOnTrue(intakeRoller.intakeToggleCommand());

    operator
        .rightTrigger(0.5)
        .toggleOnTrue(
            ShooterCommands.shootWithWhipAndShake(
                shooter, whip, kicker, intakeDeploy, led, Constants.DEMO_RPS));

    // operator
    // .y()
    // .whileTrue(
    // ShooterCommands.shootWithWhipAndShake(shooter, whip, kicker, intakeDeploy, led, 48.0));
    // 48.0));
  }

  // ============================================================
  // ENABLE HOMING
  // ============================================================
  // public void enableHoming() {
  // intakeDeploy.startHoming();
  // CommandScheduler.getInstance().schedule(climb.homeCommand());
  // }

  public Command enableHomingCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> intakeDeploy.startHoming()), climb.homeCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();

    // return Commands.sequence(

    // ClimbCommands.climbUp(climb).withTimeout(2.0))
    // .withName("ClimbOnlyAuto");
  }

  // ============================================================
  // PERIODIC (VISION + RUMBLE)
  // ============================================================
  public void periodic() {

    // ---------------- ENDGAME RUMBLE (FIXED) ----------------
    double matchTime = DriverStation.getMatchTime();

    boolean validMatch = DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled();

    boolean alert20 = validMatch && matchTime > 0 && matchTime <= endgameAlert1.get();
    boolean alert10 = validMatch && matchTime > 0 && matchTime <= endgameAlert2.get();

    double rumble = (alert20 || alert10) ? 0.6 : 0.0;

    driver.getHID().setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kBothRumble, rumble);

    // ---------------- LOGGER ----------------
    Logger.recordOutput("Match/Alert20", alert20);
    Logger.recordOutput("Match/Alert10", alert10);
  }
}
