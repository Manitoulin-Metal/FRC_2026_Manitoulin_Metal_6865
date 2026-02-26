
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Unit;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import edu.wpi.first.units.TimeUnit;

public class LEDSubsystem extends SubsystemBase {
  CANdle candle = new CANdle(4, "DriveCanivore"); // makes a new candle with ID 0

  private static final int kPort = 9;
  private static final int kLength = 120;

  // private final AddressableLED m_led;
  // private final AddressableLEDBuffer m_buffer;
  // private final AddressableLEDBuffer m_ledBuffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    Object LEDStripType;

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);

    // m_led = new AddressableLED(kPort);
    // m_buffer = new AddressableLEDBuffer(kLength);
    // m_led.setLength(kLength);
    // m_led.start();
    // m_ledBuffer = new AddressableLEDBuffer(60);
    // m_led.setLength(m_ledBuffer.getLength());
    // m_led.setData(m_ledBuffer);
    // m_led.start();

    AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);
    /*
     * To be used if needed for seperate sections:
     * AddressableLEDBufferView m_left = m_buffer.createView(0, 59);
     * AddressableLEDBufferView m_right = m_buffer.createView(60, 119).reversed();
     */
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command LEDCommand(String color) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
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
            // case "team_pattern_1":
            // TEAM_PATTERN1(null, null, null, null);
            // break;
            // case "team_pattern_2":
            // TEAM_PATTERN2(null, null, null, null);
            // break;
            // case "rainbow":
            // RAINBOW(null, null, null, null);
            // break;
            default:
              OFF();
              break;
          }
        });
  }

  // public Command runPattern(LEDPattern pattern)
  // {
  // return run(
  // // () -> pattern.applyTo(m_buffer)
  // );
  // }

  public void RED() {
    SmartDashboard.putString("Candle Colour: ", "Red");
    candle.setLEDs(255, 0, 0);
  }

  public void BLUE() {
    SmartDashboard.putString("Candle Colour: ", "Blue");
    candle.setLEDs(0, 0, 255);
  }

  public void GREEN() {
    SmartDashboard.putString("Candle Colour: ", "Green");
    candle.setLEDs(0, 255, 0);
  }

  public void YELLOW() {
    SmartDashboard.putString("Candle Colour: ", "Yellow");
    candle.setLEDs(255, 255, 0);
  }

  public void PURPLE() {
    SmartDashboard.putString("Candle Colour: ", "Purple");
    candle.setLEDs(160, 32, 240);
  }

  public void ORANGE() {
    SmartDashboard.putString("Candle Colour: ", "Orange");
    candle.setLEDs(255, 165, 0);
  }

  public void OFF() {
    SmartDashboard.putString("Candle Colour: ", "Off");
    candle.setLEDs(255, 255, 255);
  }

  // public void TEAM_PATTERN1(Unit Meters, Unit Second, Unit Percent, Unit
  // Centimeters)
  // {
  // Distance ledSpacing = (Distance) Meters.of(1 / 120.0);
  // LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
  // Color.kYellow, Color.kBlack);
  // LEDPattern pattern = base.scrollAtRelativeSpeed((Frequency)
  // Percent.per((TimeUnit) Second).of(25));
  // LEDPattern absolute = base.scrollAtAbsoluteSpeed((LinearVelocity)
  // Centimeters.per((TimeUnit) Second).of(12.5), ledSpacing);
  // //Activation
  // pattern.applyTo(m_ledBuffer);
  // m_led.setData(m_ledBuffer);
  // }

  // public void TEAM_PATTERN2(Unit Meters, Unit Second, Unit Percent, Unit
  // Seconds)
  // {
  // LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
  // Color.kYellow, Color.kBlack);
  // LEDPattern pattern = base.breathe((Time) Seconds.of(2));
  // //Activation
  // pattern.applyTo(m_ledBuffer);
  // m_led.setData(m_ledBuffer);
  // }

  // public void RAINBOW(Unit Meters, Unit Second, Unit Percent, Unit Centimeters)
  // {
  // Distance ledSpacing = (Distance) Meters.of(1 / 120.0);
  // LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
  // Color.kRed, Color.kPurple);
  // LEDPattern pattern = base.scrollAtRelativeSpeed((Frequency)
  // Percent.per((TimeUnit) Second).of(25));
  // LEDPattern absolute = base.scrollAtAbsoluteSpeed((LinearVelocity)
  // Centimeters.per((TimeUnit) Second).of(12.5), ledSpacing);
  // //Activation
  // pattern.applyTo(m_ledBuffer);
  // m_led.setData(m_ledBuffer);
  // }

  public CANdle getCaNdle() {
    return candle;
  }

  /*
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
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
