// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  @SuppressWarnings({"deprecated"})
  // CANdle support - uncomment when CTRE APIs are available
  CANdle candle = new CANdle(4, "DriveCanivore");

  private static final int kPort = 9;
  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_ledBuffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command LEDCommand(String color) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          switch (color) {
            case "red":
              RED();
              break;
            case "green":
              GREEN();
              break;
            case "blue":
              BLUE();
              break;
            case "yellow":
              YELLOW();
              break;
            case "purple":
              PURPLE();
              break;
            case "orange":
              ORANGE();
              break;
            case "team_pattern_1":
              TEAM_PATTERN1();
              break;
            case "team_pattern_2":
              TEAM_PATTERN2();
              break;
            case "rainbow":
              RAINBOW();
              break;
            default:
              OFF();
              break;
          }
        });
  }

  public Command runPattern(LEDPattern pattern) {
    return runOnce(() -> pattern.applyTo(m_ledBuffer));
  }

  public void RED() {
    SmartDashboard.putString("Candle Colour: ", "Red");
    setAllLEDs(255, 0, 0);
  }

  public void BLUE() {
    SmartDashboard.putString("Candle Colour: ", "Blue");
    setAllLEDs(0, 0, 255);
  }

  public void GREEN() {
    SmartDashboard.putString("Candle Colour: ", "Green");
    setAllLEDs(0, 255, 0);
  }

  public void YELLOW() {
    SmartDashboard.putString("Candle Colour: ", "Yellow");
    setAllLEDs(255, 255, 0);
  }

  public void PURPLE() {
    SmartDashboard.putString("Candle Colour: ", "Purple");
    setAllLEDs(160, 32, 240);
  }

  public void ORANGE() {
    SmartDashboard.putString("Candle Colour: ", "Orange");
    setAllLEDs(255, 165, 0);
  }

  public void OFF() {
    SmartDashboard.putString("Candle Colour: ", "Off");
    setAllLEDs(0, 0, 0);
  }

  private void setAllLEDs(int r, int g, int b) {
    var color = new Color(r, g, b);
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
  }

  public void TEAM_PATTERN1() {
    // Simple yellow pattern with alternating LEDs
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_ledBuffer.setLED(i, Color.kYellow);
      } else {
        m_ledBuffer.setLED(i, Color.kBlack);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void TEAM_PATTERN2() {
    // Yellow gradient pattern
    LEDPattern pattern =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kYellow, Color.kBlack);
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void RAINBOW() {
    // Rainbow pattern
    LEDPattern base = LEDPattern.rainbow(255, 128);
    base.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  /** Flashing red pattern for gyro disconnected alert */
  public void gyroDisconnectedAlert() {
    // Alternate between red and off every 500ms
    boolean isRedPhase = (int) (System.currentTimeMillis() / 500) % 2 == 0;
    if (isRedPhase) {
      setAllLEDs(255, 0, 0); // Red
    } else {
      setAllLEDs(0, 0, 0); // Off
    }
  }

  /*
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
