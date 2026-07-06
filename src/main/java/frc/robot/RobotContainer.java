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
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import java.util.Optional;
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

  private final RobotVisualizer visualizer = new RobotVisualizer(intakeDeploy, climb);

  // ============================================================
  // VISION
  // ============================================================
  private final Vision vision;
  private boolean visionEnabled = true;

  // ============================================================
  // FIELD / AUTO
  // ============================================================
  private final Field2d field = new Field2d();

  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final LoggedDashboardChooser<Command> autoChooser;

  // ============================================================
  // ENDGAME CONFIG
  // ============================================================

  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("/Tuning/Endgame Alert 20", 20.0);

  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("/Tuning/Endgame Alert 10", 10.0);

  // ============================================================
  // ENDGAME STATE MACHINE
  // ============================================================

  private enum EndgameState {
    NONE,
    WARNING_20,
    WARNING_10,
    CRITICAL
  }

  private EndgameState endgameState = EndgameState.NONE;

  // ============================================================
  // CONSTRUCTOR
  // ============================================================
  public RobotContainer() {

    // ---------------- DRIVE INIT ----------------
    drive = createDrive();

    // ---------------- VISION INIT (CLEAN) ----------------

    vision = new Vision(
        drive::addVisionMeasurement,
        () -> drive.getPose(),
        new VisionIOLimelight(VisionConstants.rearCameraName, drive::getRotation),
        new VisionIOLimelight(VisionConstants.frontCameraName, drive::getRotation));

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
    };
  }

  // ============================================================
  // NAMED COMMANDS
  // ============================================================
  private void registerNamedCommands() {

    NamedCommands.registerCommand("StopDrive", Commands.runOnce(drive::stop, drive));

    NamedCommands.registerCommand(
        "IntakeDeploy", Commands.runOnce(intakeDeploy::deploy, intakeDeploy));

    NamedCommands.registerCommand("IntakeStow", Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    // NamedCommands.registerCommand("ClimbAutoUp",
    // ClimbCommands.climbUp(climb).withTimeout(2.0));

    // wraps command with a check to see if
    // climber is homed before raising hook
    // NamedCommands.registerCommand(
    // "ClimbAutoUp",
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
          Optional<Pose2d> opt = vision.getDockTransform().map(t -> drive.getPose().plus(t));

          if (opt.isEmpty()) {
            return;
          }

          Pose2d tagPose = opt.get();

          // rest of your logic...
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

    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    
    driver.y().whileTrue(DriveCommands.dockToClimb(drive, vision));

    driver.a().whileTrue(DriveCommands.logDockCalibration(vision));

    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    // ---------------- OPERATOR ----------------
    operator.a().onTrue(Commands.runOnce(intakeDeploy::deploy, intakeDeploy));
    operator.b().onTrue(Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    operator.pov(0).onTrue(Commands.runOnce(climb::moveDown, climb));
    operator.pov(180).onTrue(Commands.runOnce(climb::moveUp, climb));
    operator.pov(0).onFalse(Commands.runOnce(climb::stop, climb));
    operator.pov(180).onFalse(Commands.runOnce(climb::stop, climb));

    operator.leftTrigger(0.1).toggleOnTrue(intakeRoller.intakeToggleCommand());

    operator
        .rightTrigger(0.5)
        .toggleOnTrue(
            ShooterCommands.shootWithWhipAndShake(
                shooter, whip, kicker, intakeDeploy, led, Constants.DEMO_RPS));
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
  }

  public RobotVisualizer getVisualizer() {
    return visualizer;
  }

  // ============================================================
  // PERIODIC
  // ============================================================
  public void periodic() {

    visualizer.update();
    double matchTime = DriverStation.getMatchTime();
    boolean active = DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled();

    EndgameState newState = EndgameState.NONE;

    if (active && matchTime > 0) {
      if (matchTime <= 10) {
        newState = EndgameState.CRITICAL;
      } else if (matchTime <= 20) {
        newState = EndgameState.WARNING_10;
      } else {
        newState = EndgameState.WARNING_20;
      }
    }

    if (newState != endgameState) {
      endgameState = newState;

      Logger.recordOutput("Match/EndgameState", endgameState.toString());

      switch (endgameState) {
        case WARNING_20 -> {
          led.requestState(LEDSubsystem.LEDState.ENDGAME);
        }

        case WARNING_10 -> {
          led.requestState(LEDSubsystem.LEDState.ENDGAME);
          driver
              .getHID()
              .setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kBothRumble, 0.4);
        }

        case CRITICAL -> {
          led.requestState(LEDSubsystem.LEDState.ENDGAME);
          driver
              .getHID()
              .setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kBothRumble, 0.8);
        }

        case NONE -> {
          driver
              .getHID()
              .setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kBothRumble, 0.0);
          led.clearToDefault();
        }
      }
    }

    Logger.recordOutput("Match/Alert20", endgameState == EndgameState.WARNING_20);
    Logger.recordOutput("Match/Alert10", endgameState == EndgameState.WARNING_10);
    Logger.recordOutput("Match/CRITICAL", endgameState == EndgameState.CRITICAL);
  }
}
