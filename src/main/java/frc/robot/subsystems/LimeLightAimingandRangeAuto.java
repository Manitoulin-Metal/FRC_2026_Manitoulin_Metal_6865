package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.CommandXboxController;

public class LimeLightAimingandRangeAuto {
  private static final double KpAim = -0.1;
  private static final double KpDistance = -0.1;
  private static final double min_aim_command = 0.05;

  private final NetworkTable telemetryTable =
      NetworkTableInstance.getDefault().getTable("limelight");

      NetworkTableInstance.getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    



  public double[] getAimCommands(CommandXboxController joystick) {
    double tx = telemetryTable.getEntry("tx").getDouble(0.0);
    double ty = telemetryTable.getEntry("ty").getDouble(0.0);

    double left_command = 0.0;
    double right_command = 0.0;

    if (joystick.a().getAsBoolean()) {
      double heading_error = -tx;
      double distance_error = -ty;
      double steering_adjust = 0.0;

      if (tx > 1.0) {
        steering_adjust = KpAim * heading_error - min_aim_command;
      } else if (tx < -1.0) {
        steering_adjust = KpAim * heading_error + min_aim_command;
      }

      double distance_adjust = KpDistance * distance_error;

      left_command += steering_adjust + distance_adjust;
      right_command -= steering_adjust + distance_adjust;
    }

    return new double[] {left_command, right_command};
  }
}
