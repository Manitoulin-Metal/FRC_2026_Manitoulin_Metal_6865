package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdle;

public class LEDCANdle extends CANdle {

  @SuppressWarnings("removal")
  public LEDCANdle(int deviceId, String canbus) {
    super(deviceId, canbus);
  }
}
