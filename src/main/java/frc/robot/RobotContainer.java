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
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Whip.WhipSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.intakedeploy.IntakeDeploySubsystem;
import frc.robot.subsystems.intake.intakeroller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
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
    visionClimb =
        new VisionSubsystem(new VisionIOLimelight("limelight", drive::getRotation), drive);

    visionShoot =
        new VisionSubsystem(new VisionIOLimelight("limelight_forward", drive::getRotation), drive);

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
    NamedCommands.registerCommand(
        "ClimbAutoUp",
        DriveCommands.driveToClimb(
            drive,
            fieldLayout,
            new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
            1.5,
            3.0));
    // IMEDIANTELY UNCOMMENT THIS
    // NamedCommands.registerCommand(
    // "ClimbAutoUp", Commands.runOnce(() ->
    // climb1.ClimbCommand(0.5).withTimeout(4).schedule()));

    NamedCommands.registerCommand("ClimbAutoDown", climb1.climbCommand(-0.5).withTimeout(6));

    NamedCommands.registerCommand(
        "timedShootCommand",
        Commands.parallel(
                Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
                kicker.kickerCommand())
            .withTimeout(1.5)
            .andThen(shooter.stopCommand(), kicker.stopCommand()));

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

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotCentric = !robotCentric;
                  SmartDashboard.putBoolean("Drive/RobotCentric", robotCentric);
                }));

    driver
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  visionEnabled = !visionEnabled;
                  SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
                }));

    // Intake deploy/stow
    // controller1.a().onTrue(intakeDeploy.deployCommand());
    // controller1.b().onTrue(intakeDeploy.stowCommand());

    // New intake deploy
    operator.a().onTrue(Commands.runOnce(intakeDeploy::deploy, intakeDeploy));
    // New intake stow
    operator.b().onTrue(Commands.runOnce(intakeDeploy::stow, intakeDeploy));

    // Stop drive (X)
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Gyro reset
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    // Drive to Shoot Command
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

    // Vision drive to fieldtag (shooting)
    /*
     * controller
     * .rightTrigger(0.5)
     * .whileTrue(
     * DriveCommands.driveToShoot(
     * drive,
     * fieldLayout,
     * 25, // example tag ID
     * 0.5, // distance
     * 1.5,
     * 3.0));
     */

    // Debug offset calc
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  int tagId = 32;

                  var tagPose = fieldLayout.getTagPose(tagId).get().toPose2d();
                  var robotPose = drive.getPose();

                  Transform2d offset = new Transform2d(tagPose, robotPose);

                  SmartDashboard.putNumber("ClimbOffset/X", offset.getX());
                  SmartDashboard.putNumber("ClimbOffset/Y", offset.getY());
                  SmartDashboard.putNumber("ClimbOffset/RotDeg", offset.getRotation().getDegrees());
                }));

    // Climb controls
    operator.pov(0).whileTrue(climb1.climbCommand(0.75)).onFalse(climb1.climbCommand(0));
    operator.pov(180).whileTrue(climb1.climbCommand(-0.75)).onFalse(climb1.climbCommand(0));

    // Intake controls
    operator
        .leftTrigger(0.1)
        .whileTrue(intakeRoller.intakeCommand())
        .onFalse(intakeRoller.idleCommand());

    // Shooter controls
    operator
        .rightTrigger(0.5)
        .whileTrue(
            Commands.parallel(
                Commands.run(() -> shooter.runShooter(75.0), shooter), whip.whipCommand()))
        .onFalse(shooter.stopCommand())
        .toggleOnFalse(whip.whipStopCommand());

    // Shooter Slow controls
    operator
        .y()
        .whileTrue(Commands.run(() -> shooter.runShooter(60.0), shooter))
        .onFalse(shooter.stopCommand());

    // Whip
    operator.rightBumper().toggleOnTrue(whip.whipCommand());

    // Kicker test
    driver.leftTrigger(0.5).whileTrue(kicker.kickerCommand());

    // Vision for Climb
    driver.leftBumper().whileTrue(driveToClimbVision());

    // Shooter faults clear
    operator.leftBumper().onTrue(shooter.clearFaultsCommand());
  }

  public Command driveToClimbVision() {
    return Commands.run(
        () -> {
          double forward = 0;
          double strafe = 0;
          double turn = 0;

          if (!visionClimb.shouldUseVisionForClimb()) {
            // SEARCH MODE
            turn = 0.5;
          } else {

            var errorOpt = visionClimb.getRobotRelativeError();

            if (errorOpt.isEmpty()) {
              drive.stop();
              return;
            }

            Transform2d error = errorOpt.get();

            forward = error.getX() * Constants.CLIMB_kP_FORWARD;
            strafe = error.getY() * Constants.CLIMB_kP_STRAFE;
            turn = error.getRotation().getRadians() * Constants.CLIMB_kP_TURN;

            // Deadbands
            if (Math.abs(error.getX()) < 0.5) forward = 0;
            if (Math.abs(error.getY()) < 0.5) strafe = 0;
            if (Math.abs(error.getRotation().getDegrees()) < 1.0) turn = 0;
          }

          // Clamp speeds
          forward = MathUtil.clamp(forward, -1.0, 1.0);
          strafe = MathUtil.clamp(strafe, -1.0, 1.0);
          turn = MathUtil.clamp(turn, -1.0, 1.0);

          // ✅ ROBOT-CENTRIC DRIVE
          drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
        },
        drive);
  }

  public Command limelightClimbFull() {
    return Commands.run(
        () -> {
          boolean seesTag = visionClimb.hasTag(Constants.CLIMB_TAG_ID);

          double forward = 0;
          double strafe = 0;
          double turn = 0;

          if (!seesTag) {
            // 🔍 SEARCH MODE
            turn = 0.5;
            forward = 0;
            strafe = 0;
          } else {

            double tx = visionClimb.getTX();
            double ty = visionClimb.getTY();

            // 🎯 TARGETS (YOU MEASURED THIS!)
            double targetTX = 0.0;
            double targetTY = 9.15;

            // 🎮 GAINS (safe starting point)
            double kTurn = 0.035;
            double kForward = 0.08;
            double kStrafe = 0.025;

            double errorX = targetTX - tx;
            double errorY = targetTY - ty;

            // Controls
            turn = errorX * kTurn;
            forward = errorY * kForward;
            strafe = errorX * kStrafe;

            // Deadbands = stability
            if (Math.abs(errorX) < 1.0) {
              turn = 0;
              strafe = 0;
            }

            if (Math.abs(errorY) < 0.5) {
              forward = 0;
            }
          }

          // Clamp speeds (VERY IMPORTANT FOR TESTING)
          turn = MathUtil.clamp(turn, -1.0, 1.0);
          forward = MathUtil.clamp(forward, -1.0, 1.0);
          strafe = MathUtil.clamp(strafe, -1.0, 1.0);

          drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
        },
        drive);
  }

  // Agitator
  // controller1.x().onTrue(intakeDeploy.deployAgitatorCommand());

  // ---------- ENABLE HOMING METHOD ----------
  public void enableHoming() {
    intakeDeploy.startHoming();
  }

  // ============================================================
  // -------------------- AUTO ----------------------------------
  // ============================================================

  public static Command driveToShootVision(
      Drive drive,
      VisionSubsystem vision,
      AprilTagFieldLayout fieldLayout,
      Supplier<Boolean> visionEnabled,
      Supplier<Double> driverX,
      Supplier<Double> driverY,
      Supplier<Double> driverRot,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
        () -> {

          // =========================
          // DRIVER OVERRIDE (NO VISION)
          // =========================
          if (!visionEnabled.get()) {
            drive.runVelocity(new ChassisSpeeds(driverX.get(), driverY.get(), driverRot.get()));
            return;
          }

          // =========================
          // AUTO TAG SELECTION
          // =========================
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          int targetTag = (alliance == Alliance.Blue) ? 25 : 9;

          // If we don't see tag → fallback to driver
          if (!vision.hasTag(targetTag)) {
            drive.runVelocity(new ChassisSpeeds(driverX.get(), driverY.get(), driverRot.get()));
            return;
          }

          // =========================
          // GET TAG POSE
          // =========================
          Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(targetTag);
          if (tagPose3d.isEmpty()) return;

          Pose2d tagPose = tagPose3d.get().toPose2d();
          Pose2d robotPose = drive.getPose();

          // =========================
          // 2m SHOOTING ARC TARGET
          // =========================
          Transform2d offset =
              new Transform2d(
                  new Translation2d(-2.0, 0.0), // 2m back from tag
                  Rotation2d.fromDegrees(180) // face target
                  );

          Pose2d targetPose = tagPose.transformBy(offset);

          // =========================
          // ERROR CALCULATION
          // =========================
          Transform2d error = targetPose.minus(robotPose);

          double forwardVision = error.getX() * kPLinear;
          double strafeVision = error.getY() * kPLinear;
          double rotVision = error.getRotation().getRadians() * kPRotation;

          // =========================
          // DISTANCE + RPM LOGIC
          // =========================
          double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());

          double targetRPM = Constants.getRPMForDistance(distance);

          SmartDashboard.putString(
              "Shooter Status",
              String.format("Shooting to %.2f m at %.0f RPM", distance, targetRPM));

          SmartDashboard.putNumber("Shooter/DistanceToTarget", distance);
          SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);

          // =========================
          // DRIVER + VISION BLENDING
          // =========================
          double visionWeight = 0.7;
          double driverWeight = 0.3;

          double vx = driverX.get() * driverWeight + forwardVision * visionWeight;
          double vy = driverY.get() * driverWeight + strafeVision * visionWeight;
          double vr = driverRot.get() * driverWeight + rotVision * visionWeight;

          // =========================
          // CLAMP SPEEDS
          // =========================
          double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
          double maxAngular = drive.getMaxAngularSpeedRadPerSec();

          vx = MathUtil.clamp(vx, -maxLinear, maxLinear);
          vy = MathUtil.clamp(vy, -maxLinear, maxLinear);
          vr = MathUtil.clamp(vr, -maxAngular, maxAngular);

          // =========================
          // DRIVE
          // =========================
          drive.runVelocity(new ChassisSpeeds(vx, vy, vr));
        },
        drive);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // ============================================================
  // -------------------- PERIODIC ------------------------------
  // ============================================================

  public void periodic() {
    Logger.recordOutput("Controls/DriverLeftY", driver.getLeftY());
    Logger.recordOutput("Controls/DriverLeftX", driver.getLeftX());
    Logger.recordOutput("Controls/DriverRightX", driver.getRightX());
    Logger.recordOutput("Controls/DriverRT", driver.getRightTriggerAxis());
    Logger.recordOutput("Controls/DriverLT", driver.getLeftTriggerAxis());
    Logger.recordOutput("Controls/OperatorLT", operator.getLeftTriggerAxis());
    Logger.recordOutput("Controls/OperatorRT", operator.getRightTriggerAxis());

    double matchTime = DriverStation.getMatchTime();

    boolean alert20 = matchTime > 0 && matchTime <= endgameAlert1.get();
    boolean alert10 = matchTime > 0 && matchTime <= endgameAlert2.get();

    SmartDashboard.putBoolean("Endgame 20s", alert20);
    SmartDashboard.putBoolean("Endgame 10s", alert10);

    Logger.recordOutput("Match/Endgame20", alert20);
    Logger.recordOutput("Match/Endgame10", alert10);

    // ✅ Define rumble HERE (inside method, before use)
    double rumble = (alert20 || alert10) ? 0.5 : 0.0;

    // ✅ Apply rumble using HID
    driver.getHID().setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kLeftRumble, rumble);

    driver.getHID().setRumble(edu.wpi.first.wpilibj.XboxController.RumbleType.kRightRumble, rumble);
    // LED + gyro alerts
    if (drive.isGyroDisconnected()) {
      led.gyroDisconnectedAlert();
    }

    // Vision diagnostics
    double tx = visionClimb.getTX(); // Limelight horizontal angle
    double ty = visionClimb.getTY(); // Limelight vertical

    SmartDashboard.putNumber("CameraToTag/measuredTX", tx);
    SmartDashboard.putNumber("CameraToTag/measuredTY", ty);

    double[] offsets =
        visionClimb.getCameraToTagOffset(fieldLayout, Constants.CLIMB_TAG_ID, tx, ty);

    SmartDashboard.putNumber("CameraToTag/X", offsets[0]);
    SmartDashboard.putNumber("CameraToTag/Y", offsets[1]);
    SmartDashboard.putNumber("CameraToTag/Distance", offsets[2]);

    // Controller diagnostics
    SmartDashboard.putNumber("Driver/LeftY", driver.getLeftY());
    SmartDashboard.putNumber("Operator/LeftY", operator.getLeftY());
    SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
  }
}
