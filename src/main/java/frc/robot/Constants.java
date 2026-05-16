// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// Modified by Team 6865, Manitoulin Metal

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.Climb.Motion;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class Constants {

  // =========================================================
  // FIELD
  // =========================================================

  public static final class Field {
    public static final double LENGTH_METERS = 16.54;
    public static final double WIDTH_METERS = 8.23;
  }

  // =========================================================
  // RUNTIME MODE
  // =========================================================

  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // =========================================================
  // DRIVE ALIGNMENT
  // =========================================================

  public static final double Y_ALIGN_P = 0.0;
  public static final double X_ALIGN_P = 0.0;
  public static final double ROT_ALIGN_P = 4.0;

  // =========================================================
  // SHOOTER
  // =========================================================

  public static final double SHOOTER_RPS = 28.0;
  public static final double KICKER_TRIGGER_RPS = 22.0;
  public static final double WHIP_TRIGGER_RPS = 20.0;

  public static final double SHOOTER_AT_TARGET_TOLERANCE_RPS = 3.0;
  public static final double SHOOTER_MIN_RPS = 28.0;

  public static final double AUTO_SHOOT_RPS = 28.0;

  public static final double WHIP_SLOW_SPEED = -0.15;

  public static final double TEST_SLOWSHOOTER_RPS = 6.0;

  public static final double SLOWSHOOTER_VELOCITY_RPS = TEST_SLOWSHOOTER_RPS;

  public static final double SLOWSHOOTER_KICKER_RPM_THRESHOLD = 1600.0;

  public static final int[] SHOOTING_TAG_IDS = {25, 26};

  public static final double MIN_SHOOT_DISTANCE_METERS = 1.5;
  public static final double MAX_SHOOT_DISTANCE_METERS = 5.5;

  // =========================================================
  // SHOOTER DISTANCE TABLE
  // =========================================================

  public static double getRPMForDistance(double distance) {

    if (distance < 2.40) {
      return 3550;
    }

    return 4200;
  }

  public static final double[] SHOOTER_DISTANCE_BREAKPOINTS_METERS = {1.5, 2.5, 3.5, 4.5, 5.5};

  public static final double[] SHOOTER_TARGET_RPS_BY_DISTANCE = {65.0, 68.0, 70.0, 72.0, 75.0};

  // =========================================================
  // AUTO / VISION
  // =========================================================

  public static final int HUB_TAG_ID = 25;

  public static final double AUTO_VISION_KP_LINEAR = 2.0;
  public static final double AUTO_VISION_KP_ANGULAR = 4.0;

  public static final double AUTO_BUMP_ERROR_METERS = 0.5;
  public static final double AUTO_BUMP_YAW_DEG = 10.0;

  // =========================================================
  // SHOOTER PID
  // =========================================================

  public static class Shooter {

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kV = 0.12;
    public static final double kS = 0.1;

    public static LoggedNetworkNumber kPEntry = new LoggedNetworkNumber("Tuning/Shooter/kP", kP);

    public static LoggedNetworkNumber kIEntry = new LoggedNetworkNumber("Tuning/Shooter/kI", kI);

    public static LoggedNetworkNumber kDEntry = new LoggedNetworkNumber("Tuning/Shooter/kD", kD);

    public static LoggedNetworkNumber kVEntry = new LoggedNetworkNumber("Tuning/Shooter/kV", kV);

    public static LoggedNetworkNumber kSEntry = new LoggedNetworkNumber("Tuning/Shooter/kS", kS);
  }

  // =========================================================
  // INTAKE DEPLOY
  // =========================================================

  public static final class IntakeDeploy {

    // Hardware
    public static final int MOTOR_ID = 59;
    public static final int HALL_SENSOR_PORT = 9;

    public static final double GEAR_RATIO = 270.0;

    // Positions
    public static final double STOW_ANGLE = 0.0;
    public static final double DEPLOY_ANGLE = 85.5;

    public static final double SHAKE_MIN_ANGLE = 40.0;
    public static final double SHAKE_MAX_ANGLE = 50.0;

    // PID
    public static final double kP = 0.14;
    public static final double kI = 0.0;
    public static final double kD = 0.002;

    // Feedforward
    public static final double DEPLOY_FF_VOLTS = 1.5;
    public static final double STOW_FF_VOLTS = 2.0;

    // Hold voltages
    public static final double HOMING_VOLTS = 2.0;
    public static final double STOW_HOLD_VOLTS = 0.2;
    public static final double DEPLOY_HOLD_VOLTS = 0.3;

    // Safety
    public static final double POSITION_TOLERANCE = 2.0;
    public static final double MAX_OUTPUT_VOLTS = 12.0;
  }

  // =========================================================
  // INTAKE ROLLER
  // =========================================================

  public static final class Intake {

    public static final int MOTOR_ID = 58;

    public static final double KP = 0.00025;
    public static final double KFF = 0.00017;

    public static final double IDLE_RPM = 0.0;
    public static final double INTAKE_RPM = -5000.0;
    public static final double REVERSE_RPM = 2500.0;

    public static final double JAM_CURRENT_AMPS = 32.0;
    public static final double JAM_VELOCITY_RATIO = 0.55;

    public static final double STARTUP_IGNORE_TIME = 0.40;
    public static final double JAM_DETECT_TIME = 0.10;

    public static final double UNJAM_REVERSE_TIME = 0.14;
    public static final double UNJAM_FORWARD_TIME = 0.08;

    public static final double COUNT_DELAY = 0.25;
  }

  // =========================================================
  // CLIMB
  // =========================================================

  public static final class Climb {

    // =====================================================
    // HARDWARE
    // =====================================================

    public static final class Hardware {

      public static final int LIMIT_SWITCH_CHANNEL = 8;

      public static final int PRIMARY_CLIMB_TAG_ID = 32;

      public static final int[] CLIMB_TAG_IDS = {32, 16};
    }

    // =====================================================
    // MOTION
    // =====================================================

    public static final class Motion {

      public static final double UP_SPEED = 0.75;
      public static final double DOWN_SPEED = -0.75;

      public static final double HOMING_SPEED = -0.35;

      public static final double BOTTOM_ENCODER_TOLERANCE_ROTATIONS = 0.5;

      public static final double UP_TARGET_ROTATIONS = 325.0;
    }

    // =====================================================
    // VISION ALIGNMENT
    // =====================================================

    public static final class Vision {

      public static final String REAR_LIMELIGHT = VisionConstants.rearCameraName;

      // ------------------------------------------------
      // BLUE TARGETS
      // ------------------------------------------------

      public static final LoggedNetworkNumber blueTX =
          new LoggedNetworkNumber("Tuning/Climb/BlueTX", -0);

      public static final LoggedNetworkNumber blueTY =
          new LoggedNetworkNumber("Tuning/Climb/BlueTY", +0);

      // ------------------------------------------------
      // RED TARGETS
      // ------------------------------------------------

      public static final LoggedNetworkNumber redTX =
          new LoggedNetworkNumber("Tuning/Climb/RedTX", 7.35);

      public static final LoggedNetworkNumber redTY =
          new LoggedNetworkNumber("Tuning/Climb/RedTY", 13.5);

      // ------------------------------------------------
      // Dynamic alliance helpers
      // ------------------------------------------------

      public static double targetTX() {

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        return alliance == Alliance.Red ? redTX.get() : blueTX.get();
      }

      public static double targetTY() {

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        return alliance == Alliance.Red ? redTY.get() : blueTY.get();
      }

      public static Rotation2d targetYaw() {
        return Rotation2d.kZero;
      }

      // ------------------------------------------------
      // Tolerances
      // ------------------------------------------------

      public static final double TX_TOLERANCE = 1.0;

      public static final double TY_TOLERANCE = 1.0;

      public static final double YAW_TOLERANCE_RAD = Math.toRadians(2.0);

      // ------------------------------------------------
      // Speed limits
      // ------------------------------------------------

      public static final double MAX_LINEAR_SPEED = 0.8;

      public static final double MAX_ANGULAR_SPEED = 1.0;
      public static final double PRECISION_LINEAR_SPEED = 0.25;
    }

    // =====================================================
    // PID
    // =====================================================

    public static final class PID {

      public static final double kP_FORWARD = 0.9;

      public static final double kP_STRAFE = 0.9;

      public static final double kP_TURN = 0.8;
    }

    // =====================================================
    // HOMING
    // =====================================================

    public static final class Homing {

      public static final double HOMING_TIMEOUT_SECONDS = 20.0;

      public static final LoggedNetworkNumber upTargetEntry =
          new LoggedNetworkNumber("Tuning/Climb/UpTargetRotations", Motion.UP_TARGET_ROTATIONS);
    }
  }

  // =========================================================
  // MODES
  // =========================================================

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }
}
