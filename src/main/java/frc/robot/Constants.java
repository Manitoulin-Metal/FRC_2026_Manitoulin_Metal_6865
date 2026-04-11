// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// This is being used by Team 6865, Manitoulin Metal

// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always \"real\" when
 * running on a roboRIO. Change the value of \"simMode\" to switch between \"sim\" (physics sim) and
 * \"replay\" (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double Y_ALIGN_P = 0;
  public static final double X_ALIGN_P = 0;
  public static final double ROT_ALIGN_P = 4.0;
  public static final double TEST_SHOOTER_RPS = 10.0; // Temporary test speed
  public static final double SHOOTER_VELOCITY_RPS =
      TEST_SHOOTER_RPS; // Tune this RPS (~3000 RPM for test)
  public static final double SHOOTER_KICKER_RPM_THRESHOLD = 4200.0; // Lowered to 60 RPS for testing
  public static final double SHOOTER_WHIP_RPM_THRESHOLD = 3600.0; // Lowered to 60 RPS for testing
  public static final double SHOOTER_AT_TARGET_TOLERANCE_RPS = 3.0;
  public static final double SHOOTER_MIN_RPS = 50.0;
  public static final String DONT_SEE_TAG_TIMEOUT_SECS = null;

  // public static final int[] SHOOTING_TAG_IDS = {};
  public static final int[] SHOOTING_TAG_IDS = {25, 26};
  public static final double MIN_SHOOT_DISTANCE_METERS = 1.5;
  public static final double MAX_SHOOT_DISTANCE_METERS = 5.5;
  public static final double AUTO_SHOOT_RPS = 75.0;
  // public static final double KICKER_SPEED = 50;
  // public static final double SHOOTER_WHIP_RPM_THRESHOLD = 4000.0; // superseded
  // by new threshold
  // above

  public static final double WHIP_SLOW_SPEED = -0.15;
  public static final double TEST_SLOWSHOOTER_RPS = 6.0; // Temporary test speed
  public static final double SLOWSHOOTER_VELOCITY_RPS =
      TEST_SLOWSHOOTER_RPS; // Tune this RPS (~60 RPM for test)
  public static final double SLOWSHOOTER_KICKER_RPM_THRESHOLD =
      3200.0; // Triggers kicker at shooter 60.0 RPS (1)

  public static double getRPMForDistance(double distance) {
    if (distance < 2.40) return 3550;
    return 4200;
  }

  // Vision auto positioning
  public static final int HUB_TAG_ID = 25;
  // public static final int HUB_TAG_ID = 0;
  public static final double AUTO_VISION_KP_LINEAR = 2.0;
  public static final double AUTO_VISION_KP_ANGULAR = 4.0;

  // Bump correction thresholds for autonomous
  public static final double AUTO_BUMP_ERROR_METERS = 0.5;
  public static final double AUTO_BUMP_YAW_DEG = 10.0;

  // Dynamic shooter RPM by distance to shooting tags
  public static final double[] SHOOTER_DISTANCE_BREAKPOINTS_METERS = {1.5, 2.5, 3.5, 4.5, 5.5};
  public static final double[] SHOOTER_TARGET_RPS_BY_DISTANCE = {65.0, 68.0, 70.0, 72.0, 75.0};

  public static class Shooter {
    public static final double kP = 0.01; // Minimal PID enabled for better tracking
    public static final double kI = 0.0; // Disabled (set 0) to troubleshoot not moving
    public static final double kD = 0.0; // Disabled (set 0) to troubleshoot not moving
    public static final double kV = 0.12;
    public static final double kS = 0.1;
    public static LoggedNetworkNumber kPEntry = new LoggedNetworkNumber("Tuning/Shooter/kP", kP);
    public static LoggedNetworkNumber kIEntry = new LoggedNetworkNumber("Tuning/Shooter/kI", kI);
    public static LoggedNetworkNumber kDEntry = new LoggedNetworkNumber("Tuning/Shooter/kD", kD);
    public static LoggedNetworkNumber kVEntry = new LoggedNetworkNumber("Tuning/Shooter/kV", kV);
    public static LoggedNetworkNumber kSEntry = new LoggedNetworkNumber("Tuning/Shooter/kS", kS);
  }

  public static final class IntakeDeploy {

    // ---------------- Hardware ----------------
    public static final int MOTOR_ID = 59; // <-- replace with CAN ID
    public static final int HALL_SENSOR_PORT = 9; // <-- DIO port
    public static final double GEAR_RATIO = 270.0;

    // ---------------- Positions ----------------
    public static final double STOW_ANGLE = 0.0;
    public static final double DEPLOY_ANGLE = 85.5;
    public static final double SHAKE_MIN_ANGLE = 40.0;
    public static final double SHAKE_MAX_ANGLE = 50.0;

    // ---------------- PID ----------------
    public static final double kP = 0.14;
    public static final double kI = 0.0;
    public static final double kD = 0.002;

    // ---------------- Feedforward ----------------
    public static final double DEPLOY_FF_VOLTS = 1.5; // voltage to counteract gravity during deploy
    public static final double STOW_FF_VOLTS = 2; // voltage to help lift arm when stowing

    public static final double POSITION_TOLERANCE = 2.0;

    // ---------------- Hold Voltages ----------------
    public static final double HOMING_VOLTS = 2;
    public static final double STOW_HOLD_VOLTS = 0.2;
    public static final double DEPLOY_HOLD_VOLTS = 0.3;

    // ---------------- Safety ----------------
    public static final double MAX_OUTPUT_VOLTS = 12.0;
  }

  public static class Climb {
    public static final int LIMIT_SWITCH_CHANNEL = 8;
  }

  public static class ClimbVision {
    // 🎯 Rear Limelight name
    public static final String REAR_LIMELIGHT = "limelight0";

    // 🎯 Target pose in vision-space (your measured values)
    public static final double TARGET_TX = -16.10;
    public static final double TARGET_TY = 9.15;

    // 🎯 Distance target (meters, tune this!)
    public static final double TARGET_DISTANCE = 1.2;

    // 🎛️ Gains
    public static final double kP_TURN = 0.025;
    public static final double kP_FORWARD = 0.6;
    public static final double kP_STRAFE = 0.03;

    // ✅ Tolerances
    public static final double TX_TOL = 1.5;
    public static final double TY_TOL = 1.5;
    public static final double DIST_TOL = 0.15;
  }

  // ================= CLIMB =================
  public static final int CLIMB_TAG_ID = 32;
  public static final int[] CLIMB_TAG_IDS = {
    32, 16
  }; // Add any additional tags you want to use for climbing

  public static final Transform2d CLIMB_OFFSET =
      new Transform2d(
          new Translation2d(-0.65, 0.10), // 🔥 REPLACE WITH YOUR MEASURED VALUES
          Rotation2d.fromDegrees(180));

  public static final double CLIMB_POS_TOLERANCE = 0.05;
  public static final double CLIMB_ROT_TOLERANCE = Math.toRadians(3);

  public static final double CLIMB_TARGET_TY = 9.15; // YOUR measured value
  public static final double CLIMB_STRAFE_FACTOR = 0.02;

  public static final double CLIMB_kP_FORWARD = 0.08;
  public static final double CLIMB_kP_STRAFE = 0.03;
  public static final double CLIMB_kP_TURN = 0.035;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
