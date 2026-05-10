package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Climb.PID;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0;
  private static final double FF_RAMP_RATE = 0.1;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  // -----------------------------
  // AlignToTag PID live tuning (Tune These If Need Be)
  // -----------------------------
  private static final LoggedNetworkNumber alignStrafeKP =
      new LoggedNetworkNumber("/AlignToTag/Strafe/kP", 0.05);
  private static final LoggedNetworkNumber alignStrafeKI =
      new LoggedNetworkNumber("/AlignToTag/Strafe/kI", 0.0);
  private static final LoggedNetworkNumber alignStrafeKD =
      new LoggedNetworkNumber("/AlignToTag/Strafe/kD", 0.0);
  private static final LoggedNetworkNumber alignDistanceKP =
      new LoggedNetworkNumber("/AlignToTag/Distance/kP", 0.15);
  private static final LoggedNetworkNumber alignDistanceKI =
      new LoggedNetworkNumber("/AlignToTag/Distance/kI", 0.0);
  private static final LoggedNetworkNumber alignDistanceKD =
      new LoggedNetworkNumber("/AlignToTag/Distance/kD", 0.0);
  private static final LoggedNetworkNumber alignRotationKP =
      new LoggedNetworkNumber("/AlignToTag/Rotation/kP", 0.1);
  private static final LoggedNetworkNumber alignRotationKI =
      new LoggedNetworkNumber("/AlignToTag/Rotation/kI", 0.0);
  private static final LoggedNetworkNumber alignRotationKD =
      new LoggedNetworkNumber("/AlignToTag/Rotation/kD", 0.0);

  // -----------------------------
  // Tag selection
  // -----------------------------
  static int getClimbTagId() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        ? 16
        : 32;
  }

  // -----------------------------
  // TELEOP JOYSTICK DRIVE
  // -----------------------------

  // ========== NEW AS OF 10:15AM 4/12/2026 ========== \\
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Boolean> robotCentricSupplier) {

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  @SuppressWarnings("resource")
  public static Command dockToClimb(Drive drive, Vision vision) {

    PIDController forwardController = new PIDController(PID.kP_FORWARD, 0.0, 0.0);

    PIDController strafeController = new PIDController(PID.kP_STRAFE, 0.0, 0.0);

    PIDController turnController = new PIDController(PID.kP_TURN, 0.0, 0.0);

    turnController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Optional<Pose3d> poseOpt = vision.getRearTargetSpacePose();

              if (poseOpt.isEmpty()) {
                drive.stop();
                return;
              }

              Pose3d targetSpace = poseOpt.get();

              /*
               * LIMELIGHT TARGET SPACE
               *
               * X = left/right
               * Z = forward/back
               * Rotation Z = yaw
               */

              double strafeError = targetSpace.getX();

              double forwardError =
                  targetSpace.getZ() - Constants.Climb.Vision.TARGET_FORWARD_METERS;

              double yawError = targetSpace.getRotation().getZ();

              double forward = -forwardController.calculate(forwardError, 0.0);

              double strafe = -strafeController.calculate(strafeError, 0.0);

              double turn = -turnController.calculate(yawError, 0.0);

              // Clamp outputs
              forward = MathUtil.clamp(forward, -1.0, 1.0);
              strafe = MathUtil.clamp(strafe, -1.0, 1.0);
              turn = MathUtil.clamp(turn, -1.5, 1.5);

              // Scale to drivetrain max speeds
              forward *= drive.getMaxLinearSpeedMetersPerSec();
              strafe *= drive.getMaxLinearSpeedMetersPerSec();
              turn *= drive.getMaxAngularSpeedRadPerSec();

              // Robot-relative because target-space is robot-relative
              drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
            },
            drive)
        .until(
            () -> {
              Optional<Pose3d> poseOpt = vision.getRearTargetSpacePose();

              if (poseOpt.isEmpty()) {
                return false;
              }

              Pose3d targetSpace = poseOpt.get();

              double strafeError = Math.abs(targetSpace.getX());

              double forwardError =
                  Math.abs(targetSpace.getZ() - Constants.Climb.Vision.TARGET_FORWARD_METERS);

              double yawError = Math.abs(targetSpace.getRotation().getZ());

              return strafeError < 0.03 && forwardError < 0.04 && yawError < Math.toRadians(3);
            })
        .andThen(drive::stop);
  }

  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Command JoystickDrive(
      Drive drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    return joystickDrive(
        drive,
        () -> forward.getAsDouble() * 0.5,
        () -> strafe.getAsDouble() * 0.5,
        () -> rotation.getAsDouble() * 0.5,
        () -> false);
  }

  // -----------------------------
  // Generic Drive to Pose
  // -----------------------------
  public static Command driveToPose(
      Drive drive, Pose2d targetPose, double kPLinear, double kPRotation) {

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();

              double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
              double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

              double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

              double rotSpeed = rotError * kPRotation;

              // Clamp
              double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
              double maxAngular = drive.getMaxAngularSpeedRadPerSec();

              xSpeed = MathUtil.clamp(xSpeed, -maxLinear, maxLinear);
              ySpeed = MathUtil.clamp(ySpeed, -maxLinear, maxLinear);
              rotSpeed = MathUtil.clamp(rotSpeed, -maxAngular, maxAngular);

              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        .until(
            () -> {
              Pose2d current = drive.getPose();

              double dist = current.getTranslation().getDistance(targetPose.getTranslation());

              double angle =
                  Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

              return dist < 0.05 && angle < 0.05;
            })
        .andThen(drive::stop);
  }

  // -----------------------------
  // CLIMB COMMAND (uses limelight-rear)
  // -----------------------------
  public static Command driveToClimb(
      Drive drive,
      AprilTagFieldLayout fieldLayout,
      Transform2d offset,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
            () -> {
              int tagId = getClimbTagId();

              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty()) return;

              Pose2d tagPose = tagOpt.get().toPose2d();

              Pose2d targetPose = tagPose.transformBy(offset);

              Pose2d current = drive.getPose();

              double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
              double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

              double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

              double rotSpeed = rotError * kPRotation;

              double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
              double maxAngular = drive.getMaxAngularSpeedRadPerSec();

              xSpeed = MathUtil.clamp(xSpeed, -maxLinear, maxLinear);
              ySpeed = MathUtil.clamp(ySpeed, -maxLinear, maxLinear);
              rotSpeed = MathUtil.clamp(rotSpeed, -maxAngular, maxAngular);

              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        .until(
            () -> {
              int tagId = getClimbTagId();

              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty()) return false;

              Pose2d target = tagOpt.get().toPose2d().transformBy(offset);

              Pose2d current = drive.getPose();

              double dist = current.getTranslation().getDistance(target.getTranslation());

              double angle =
                  Math.abs(current.getRotation().minus(target.getRotation()).getRadians());

              return dist < 0.05 && angle < 0.05;
            })
        .andThen(drive::stop);
  }

  // -----------------------------
  // SHOOT COMMAND (no vision)
  // -----------------------------
  public static Command driveToShoot(
      Drive drive,
      AprilTagFieldLayout fieldLayout,
      int tagId,
      double distance,
      double kPLinear,
      double kPRotation) {

    return Commands.run(
            () -> {
              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty()) return;

              Pose2d tagPose = tagOpt.get().toPose2d();

              Pose2d current = drive.getPose();

              Translation2d tagToRobot = current.getTranslation().minus(tagPose.getTranslation());

              double currentDist = tagToRobot.getNorm();

              Translation2d direction = tagToRobot.div(currentDist);

              Translation2d targetTranslation =
                  tagPose.getTranslation().plus(direction.times(distance));

              Rotation2d targetRotation =
                  tagPose.getTranslation().minus(targetTranslation).getAngle();

              Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

              double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
              double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

              double rotError = targetPose.getRotation().minus(current.getRotation()).getRadians();

              double rotSpeed = rotError * kPRotation;

              double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
              double maxAngular = drive.getMaxAngularSpeedRadPerSec();

              xSpeed = MathUtil.clamp(xSpeed, -maxLinear, maxLinear);
              ySpeed = MathUtil.clamp(ySpeed, -maxLinear, maxLinear);
              rotSpeed = MathUtil.clamp(rotSpeed, -maxAngular, maxAngular);

              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        .until(
            () -> {
              Optional<Pose3d> tagOpt = fieldLayout.getTagPose(tagId);
              if (tagOpt.isEmpty()) return false;

              Pose2d target =
                  tagOpt
                      .get()
                      .toPose2d()
                      .transformBy(
                          new Transform2d(new Translation2d(distance, 0), new Rotation2d()));

              Pose2d current = drive.getPose();

              double dist = current.getTranslation().getDistance(target.getTranslation());

              return dist < 0.05;
            })
        .andThen(drive::stop);
  }

  // ============================================================
  // Vision-based shooting alignment using Limelight + distance)
  // ============================================================
  public static Command driveToShootVision(
      Drive drive,
      Vision vision,
      frc.robot.subsystems.ShooterSubsystem shooter,
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
              // VISION OVERRIDE CHECK
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

              if (!vision.hasTag(targetTag)) {
                // fallback to driver control
                drive.runVelocity(new ChassisSpeeds(driverX.get(), driverY.get(), driverRot.get()));
                return;
              }

              // =========================
              // REAL TAG POSE MATH
              // =========================
              Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(targetTag);
              if (tagPose3d.isEmpty()) return;

              Pose2d tagPose = tagPose3d.get().toPose2d();
              Pose2d robotPose = drive.getPose();

              // 2m shooting offset (arc target)
              Transform2d offset =
                  new Transform2d(new Translation2d(-2.0, 0.0), Rotation2d.fromDegrees(180));

              Pose2d targetPose = tagPose.transformBy(offset);

              Transform2d error = targetPose.minus(robotPose);

              double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());

              // RPM from your table
              double targetRPM = Constants.getRPMForDistance(distance);

              // Convert to RPS for shooter
              double targetRps = targetRPM / 60.0;

              // Send to shooter
              shooter.runShooter(targetRps);

              // Dashboard display
              SmartDashboard.putString(
                  "Shooter Status",
                  String.format("Shooting to %.2f m at %.0f RPM", distance, targetRPM));

              // =========================
              // ARC + BLENDING
              // =========================
              double visionWeight = 0.5;
              double driverWeight = 1.0 - visionWeight;

              double forwardVision = error.getX() * kPLinear;
              double strafeVision = error.getY() * kPLinear;
              double rotVision = error.getRotation().getRadians() * kPRotation;

              double vx = driverX.get() * driverWeight + forwardVision * visionWeight;
              double vy = driverY.get() * driverWeight + strafeVision * visionWeight;
              double vr = driverRot.get() * driverWeight + rotVision * visionWeight;

              // Clamp
              double maxLinear = drive.getMaxLinearSpeedMetersPerSec();
              double maxAngular = drive.getMaxAngularSpeedRadPerSec();

              vx = MathUtil.clamp(vx, -maxLinear, maxLinear);
              vy = MathUtil.clamp(vy, -maxLinear, maxLinear);
              vr = MathUtil.clamp(vr, -maxAngular, maxAngular);

              // Drive
              drive.runVelocity(new ChassisSpeeds(vx, vy, vr));
            },
            drive)
        .until(() -> Math.abs(vision.getTX()) < 1.0 && Math.abs(vision.getTY()) < 1.0)
        .andThen(drive::stop);
  }

  /** Align to AprilTag using WPILib PIDControllers and direct LimelightHelpers reads. */
  @SuppressWarnings("resource")
  public static Command alignToTag(int targetId, Drive drive, Vision vision) {
    PIDController strafeController =
        new PIDController(alignStrafeKP.get(), alignStrafeKI.get(), alignStrafeKD.get());
    PIDController distanceController =
        new PIDController(alignDistanceKP.get(), alignDistanceKI.get(), alignDistanceKD.get());
    PIDController rotationController =
        new PIDController(alignRotationKP.get(), alignRotationKI.get(), alignRotationKD.get());

    // Set tolerances for convergence (degrees)
    strafeController.setTolerance(1.0);
    distanceController.setTolerance(0.5);
    rotationController.setTolerance(1.0);

    return Commands.run(
            () -> {
              // Use the same rear limelight used for climb alignment in RobotContainer.
              final String limelightName = "limelight";

              boolean hasTarget = LimelightHelpers.getTV(limelightName);
              int fiducialId = (int) Math.round(LimelightHelpers.getFiducialID(limelightName));
              if (!hasTarget || fiducialId != targetId) {
                drive.stop();
                SmartDashboard.putBoolean("AlignTesting/TryingToAlignToTag", false);
                return;
              }

              SmartDashboard.putBoolean("AlignTesting/TryingToAlignToTag", true);

              // Update PID gains live from NetworkTables
              strafeController.setPID(
                  alignStrafeKP.get(), alignStrafeKI.get(), alignStrafeKD.get());
              distanceController.setPID(
                  alignDistanceKP.get(), alignDistanceKI.get(), alignDistanceKD.get());
              rotationController.setPID(
                  alignRotationKP.get(), alignRotationKI.get(), alignRotationKD.get());

              double tx = LimelightHelpers.getTX(limelightName);
              double ty = LimelightHelpers.getTY(limelightName);

              // For rotation, use tx angle error (original used pose yaw which is ~tx)
              double rotationError = tx; // degrees

              SmartDashboard.putNumber("AlignTesting/tx", tx);
              SmartDashboard.putNumber("AlignTesting/ty", ty);
              SmartDashboard.putNumber("AlignTesting/rotationError", rotationError);

              // PID-controlled outputs (replaces proportional gains)
              double strafe = strafeController.calculate(tx, 0.0);
              double distance = distanceController.calculate(ty, 0.0);
              double omega = rotationController.calculate(rotationError, 0.0);

              // Clamp to safe speeds (matching original)
              double maxSpeed = 6.0;
              strafe = MathUtil.clamp(strafe, -maxSpeed, maxSpeed);
              distance = MathUtil.clamp(distance, -maxSpeed, maxSpeed);
              omega = MathUtil.clamp(omega, -maxSpeed, maxSpeed);

              // Robot-centric movement (same as original)
              drive.runVelocity(
                  new ChassisSpeeds(
                      distance, // forward/backward (ty - target_ty, but target_ty=0)
                      strafe, // left/right (tx)
                      omega // rotation
                      ));

              // Log PID states
              SmartDashboard.putBoolean(
                  "Align/PIDAtSetpoint",
                  strafeController.atSetpoint()
                      && distanceController.atSetpoint()
                      && rotationController.atSetpoint());
            },
            drive)
        .withName("AlignToTag_PID");
  }

  /** Align to AprilTag using WPILib PIDControllers and VisionSubsystem. */
  @SuppressWarnings("resource")
  public static Command alignToTagWithVision(int targetId, Drive drive, Vision vision) {
    PIDController strafeController =
        new PIDController(alignStrafeKP.get(), alignStrafeKI.get(), alignStrafeKD.get());
    PIDController distanceController =
        new PIDController(alignDistanceKP.get(), alignDistanceKI.get(), alignDistanceKD.get());
    PIDController rotationController =
        new PIDController(alignRotationKP.get(), alignRotationKI.get(), alignRotationKD.get());

    // Set tolerances for convergence (degrees)
    strafeController.setTolerance(1.0);
    distanceController.setTolerance(0.5);
    rotationController.setTolerance(1.0);

    return Commands.run(
            () -> {
              // Check if correct tag is visible (replaces getFiducialID)
              if (!vision.hasTag(targetId)) {
                drive.stop();
                SmartDashboard.putBoolean("AlignTesting/TryingToAlignToTag", false);
                return;
              }

              SmartDashboard.putBoolean("AlignTesting/TryingToAlignToTag", true);

              // Update PID gains live from NetworkTables
              strafeController.setPID(
                  alignStrafeKP.get(), alignStrafeKI.get(), alignStrafeKD.get());
              distanceController.setPID(
                  alignDistanceKP.get(), alignDistanceKI.get(), alignDistanceKD.get());
              rotationController.setPID(
                  alignRotationKP.get(), alignRotationKI.get(), alignRotationKD.get());

              // Use VisionSubsystem methods (replaces direct LimelightHelpers calls)
              double tx = vision.getTX();
              double ty = vision.getTY();

              // For rotation, use tx angle error (original used pose yaw which is ~tx)
              double rotationError = tx; // degrees

              SmartDashboard.putNumber("AlignTesting/tx", tx);
              SmartDashboard.putNumber("AlignTesting/ty", ty);
              SmartDashboard.putNumber("AlignTesting/rotationError", rotationError);

              // PID-controlled outputs (replaces proportional gains)
              double strafe = strafeController.calculate(tx, 0.0);
              double distance = distanceController.calculate(ty, 0.0);
              double omega = rotationController.calculate(rotationError, 0.0);

              // Clamp to safe speeds (matching original)
              double maxSpeed = 6.0;
              strafe = MathUtil.clamp(strafe, -maxSpeed, maxSpeed);
              distance = MathUtil.clamp(distance, -maxSpeed, maxSpeed);
              omega = MathUtil.clamp(omega, -maxSpeed, maxSpeed);

              // Robot-centric movement (same as original)
              drive.runVelocity(
                  new ChassisSpeeds(
                      distance, // forward/backward (ty - target_ty, but target_ty=0)
                      strafe, // left/right (tx)
                      omega // rotation
                      ));

              // Log PID states
              SmartDashboard.putBoolean(
                  "Align/PIDAtSetpoint",
                  strafeController.atSetpoint()
                      && distanceController.atSetpoint()
                      && rotationController.atSetpoint());
            },
            drive,
            vision)
        .withName("AlignToTagWithVision_PID");
  }
}
