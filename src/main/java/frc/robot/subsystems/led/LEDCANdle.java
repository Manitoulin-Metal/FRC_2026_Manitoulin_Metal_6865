package frc.robot.subsystems.led;

import com.ctre.phoenix6.hardware.CANdle;

public class LEDCANdle extends CANdle {

  public LEDCANdle(int deviceId, String canbus) {
    super(deviceId, canbus);
  }
}
