package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {

  private Rotation2d yaw = new Rotation2d();

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = yaw;
    inputs.yawVelocityRadPerSec = 0.0;
  }

  @Override
  public void reset() {
    yaw = new Rotation2d();
  }

  public void addYawRadians(double deltaRadians) {
    yaw = yaw.plus(new Rotation2d(deltaRadians));
  }

  public void setYaw(Rotation2d newYaw) {
    yaw = newYaw;
  }
}
