// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// This is being used by Team 6865, Manitoulin Metal

// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file at the root directory of this project.

package frc.robot.commands;

// All imports between: Line 9 - Line 30
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {
    // Put all DriveCommands here
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
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

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Constructs command
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

  /**
   * Measures the velocity feedforward constants for the drive motors. This command should only be
   * used in voltage control mode.
   */
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

  // This class allows the code to recognize the Characteristics of the wheel
  // radius
  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  // Add the reusable DriveToPose command here
  /**
   * Generic Drive to Pose command. Works for SIM, Teleop, or Autonomous.
   *
   * @param drive the Drive subsystem
   * @param targetPose the Pose2d to drive to
   * @param kP simple proportional gain for translation
   * @param fieldLayout optional field layout for vision alignment
   * @param useLimelight whether to use Limelight vision updates
   */
  /**
   * Drive to a Pose with smooth rotation. Keeps the robot facing the target heading while driving.
   */
  public static Command driveToPoseWithRotation(
      Drive drive, Pose2d targetPose, double kPLinear, double kPRotation) {

    ProfiledPIDController rotationController =
        new ProfiledPIDController(
            kPRotation,
            0.0,
            0.4,
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec(), // max velocity
                drive.getMaxAngularSpeedRadPerSec() * 2)); // max accel
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();

              // Linear proportional control
              double xSpeed = (targetPose.getX() - current.getX()) * kPLinear;
              double ySpeed = (targetPose.getY() - current.getY()) * kPLinear;

              // Clamp speeds
              double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
              xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
              ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);

              // Rotation PID
              double rotSpeed =
                  rotationController.calculate(
                      current.getRotation().getRadians(), targetPose.getRotation().getRadians());
              rotSpeed =
                  MathUtil.clamp(
                      rotSpeed,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec());

              // Send velocity to drive
              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        // Stop when close enough
        .until(
            () -> {
              Pose2d current = drive.getPose();
              double distance = current.getTranslation().getDistance(targetPose.getTranslation());
              double angleError =
                  Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());
              return distance < 0.05 && angleError < 0.05;
            })
        .andThen(Commands.runOnce(drive::stop))
        // Reset controller at start
        .beforeStarting(() -> rotationController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Drive to a pose with optional vision updates. Works in teleop (button held) or autonomous.
   *
   * @param drive the Drive subsystem
   * @param targetPose the target Pose2d
   * @param kPLinear proportional gain for translation
   * @param kPRotation proportional gain for rotation
   * @param fieldLayout used in simulation to confirm tag positions
   * @param useLimelight whether to use Limelight vision updates
   */
  public static Command driveToShoot(
      Drive drive,
      Pose2d tagPose,
      double kPLinear,
      double kPRotation,
      AprilTagFieldLayout fieldLayout,
      boolean useLimelight) {

    final double desiredRadius = 2.0; // meters from tag
    final double maxAngleRad = Math.toRadians(30.0); // 60° total cone

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();

              // Optional Limelight pose update (real robot only)
              if (useLimelight
                  && LimelightHelpers.getTV("limelight")
                  && LimelightHelpers.getFiducialID("limelight") == 26) {

                var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
                if (estimate != null && estimate.pose != null) {
                  drive.addVisionMeasurement(
                      estimate.pose,
                      estimate.timestampSeconds,
                      new Matrix<N3, N1>(
                          N3.instance, N1.instance, new double[] {0.7, 0.7, 9999999}));
                }
              }

              // Direction the tag is facing outward from the structure
              Rotation2d tagForward = tagPose.getRotation().plus(Rotation2d.fromDegrees(180));

              // Vector from tag to robot
              Translation2d tagToRobot =
                  currentPose.getTranslation().minus(tagPose.getTranslation());

              // Angle of robot relative to tag
              Rotation2d robotAngle = tagToRobot.getAngle();

              // Compute angle error relative to FRONT of tag
              double angleError = MathUtil.angleModulus(robotAngle.minus(tagForward).getRadians());

              // Clamp to ±30°
              angleError = MathUtil.clamp(angleError, -maxAngleRad, maxAngleRad);

              // Final clamped angle
              Rotation2d clampedAngle = tagForward.plus(Rotation2d.fromRadians(angleError));

              // Target position on arc
              Translation2d targetTranslation =
                  new Translation2d(
                      tagPose.getX() + desiredRadius * Math.cos(clampedAngle.getRadians()),
                      tagPose.getY() + desiredRadius * Math.sin(clampedAngle.getRadians()));

              // Robot should face the tag
              Rotation2d targetRotation =
                  tagPose.getTranslation().minus(targetTranslation).getAngle();

              Pose2d adjustedTarget = new Pose2d(targetTranslation, targetRotation);

              // Proportional translation control
              double xSpeed = (adjustedTarget.getX() - currentPose.getX()) * kPLinear;
              double ySpeed = (adjustedTarget.getY() - currentPose.getY()) * kPLinear;

              double rotError =
                  adjustedTarget.getRotation().minus(currentPose.getRotation()).getRadians();
              double rotSpeed = rotError * kPRotation;

              // Clamp speeds
              xSpeed =
                  MathUtil.clamp(
                      xSpeed,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec());

              ySpeed =
                  MathUtil.clamp(
                      ySpeed,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec());

              rotSpeed =
                  MathUtil.clamp(
                      rotSpeed,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec());

              // Field-relative speeds
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              ChassisSpeeds speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed,
                      ySpeed,
                      rotSpeed,
                      isFlipped
                          ? drive.getRotation().plus(Rotation2d.fromDegrees(180))
                          : drive.getRotation());

              drive.runVelocity(speeds);
            },
            drive)
        .until(
            () -> {
              Pose2d current = drive.getPose();
              double distance = current.getTranslation().getDistance(tagPose.getTranslation());

              return Math.abs(distance - desiredRadius) < 0.05;
            })
        .andThen(Commands.runOnce(drive::stop));
  }

  /**
   * Drive to climb position relative to AprilTag 31. Uses Limelight if available (real robot) or
   * fieldLayout in simulation. Stops when the robot reaches the target pose.
   */
  public static Command driveToClimb(
      Drive drive,
      AprilTagFieldLayout fieldLayout,
      double kPLinear,
      double kPRotation,
      boolean useLimelight) {

    Rotation2d targetRotation = Rotation2d.fromDegrees(180);
    Pose2d blueTargetPose = new Pose2d(1.549, 2.978, targetRotation);

    // Flip for Red alliance in sim
    final Pose2d adjustedTarget;
    if (!useLimelight
        && fieldLayout != null
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {

      adjustedTarget =
          new Pose2d(
              fieldLayout.getFieldLength() - blueTargetPose.getX(),
              blueTargetPose.getY(),
              blueTargetPose.getRotation().plus(Rotation2d.fromDegrees(180)));

    } else {
      adjustedTarget = blueTargetPose;
    }

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();

              // Vision update (REAL ROBOT ONLY)
              if (useLimelight && LimelightHelpers.getTV("limelight")) {

                var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
                if (estimate != null && estimate.pose != null) {
                  drive.addVisionMeasurement(
                      estimate.pose,
                      estimate.timestampSeconds,
                      new Matrix<N3, N1>(
                          N3.instance, N1.instance, new double[] {0.7, 0.7, 9999999}));
                }
              }

              double xSpeed = (adjustedTarget.getX() - currentPose.getX()) * kPLinear;
              double ySpeed = (adjustedTarget.getY() - currentPose.getY()) * kPLinear;

              double rotError =
                  adjustedTarget.getRotation().minus(currentPose.getRotation()).getRadians();

              double rotSpeed = rotError * kPRotation;

              xSpeed =
                  MathUtil.clamp(
                      xSpeed,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec());

              ySpeed =
                  MathUtil.clamp(
                      ySpeed,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec());

              rotSpeed =
                  MathUtil.clamp(
                      rotSpeed,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec());

              drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            },
            drive)
        .until(
            () -> {
              Pose2d current = drive.getPose();
              double distance =
                  current.getTranslation().getDistance(adjustedTarget.getTranslation());
              double angleError =
                  Math.abs(current.getRotation().minus(adjustedTarget.getRotation()).getRadians());

              return distance < 0.05 && angleError < 0.05;
            })
        .andThen(Commands.runOnce(drive::stop));
  }
}
