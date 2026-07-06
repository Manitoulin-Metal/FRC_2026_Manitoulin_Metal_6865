package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  // =========================================================
  // CONFIG
  // =========================================================
  private static final int PWM_PORT = 9;
  private static final int LED_LENGTH = 100;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  private int tick = 0;

  // =========================================================
  // STATE MACHINE
  // =========================================================
  public enum LEDState {
    DISABLED,
    ENABLED,
    INTAKING,
    VISION_LOCK,
    SHOOTER_READY,
    SHOOTING,
    CLIMBING_UP,
    CLIMBING_DOWN,
    ENDGAME
  }

  private LEDState currentState = LEDState.DISABLED;

  // =========================================================
  // CONSTRUCTOR
  // =========================================================
  public LEDSubsystem() {
    led = new AddressableLED(PWM_PORT);
    buffer = new AddressableLEDBuffer(LED_LENGTH);

    led.setLength(buffer.getLength());
    led.start();
  }

  // =========================================================
  // PUBLIC CONTROL
  // =========================================================
  public void requestState(LEDState state) {
    // simple override model (no priority bugs)
    currentState = state;
  }

  public void clearToDefault() {
    currentState = DriverStation.isDisabled() ? LEDState.DISABLED : LEDState.ENABLED;
  }

  public LEDState getState() {
    return currentState;
  }

  // =========================================================
  // PERIODIC
  // =========================================================
  @Override
  public void periodic() {
    tick++;

    // base safety behavior
    if (DriverStation.isDisabled()) {
      currentState = LEDState.DISABLED;
    }

    // endgame override (centralized here ONLY)
    if (DriverStation.isTeleopEnabled()) {
      double t = DriverStation.getMatchTime();
      if (t > 0 && t <= 20) {
        currentState = LEDState.ENDGAME;
      }
    }

    switch (currentState) {
      case DISABLED -> beeIdle();
      case ENABLED -> queenPulse();
      case INTAKING -> nectarFlow();
      case VISION_LOCK -> visionLock();
      case SHOOTER_READY -> shooterReady();
      case SHOOTING -> stingerFire();
      case CLIMBING_UP -> rainbow(0.75);
      case CLIMBING_DOWN -> rainbow(3.0);
      case ENDGAME -> endgamePulse();
    }
  }

  // =========================================================
  // PATTERNS
  // =========================================================

  private void beeIdle() {
    for (int i = 0; i < buffer.getLength(); i++) {
      double wave = 0.5 + 0.5 * Math.sin((i * 0.18) + (tick * 0.03));

      Color c = (wave > 0.5) ? new Color(0, 0.76, 0.67) : new Color(0, 0.55, 0.45);

      buffer.setLED(i, c);
    }
    push();
  }

  private void queenPulse() {
    double p = 0.35 + 0.15 * Math.sin(tick * 0.04);
    solid(new Color(p, p * 0.75, 0.08));
  }

  private void nectarFlow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      double wave = 0.5 + 0.5 * Math.sin((i * 0.45) - (tick * 0.25));
      buffer.setLED(i, new Color(0.0, wave * 0.8, 0.05));
    }
    push();
  }

  private void visionLock() {
    double p = 0.5 + 0.5 * Math.sin(tick * 0.12);
    solid(new Color(p, p * 0.8, 0.1));
  }

  private void shooterReady() {
    double p = 0.4 + 0.4 * Math.sin(tick * 0.18);
    solid(new Color(p, p * 0.45, 0.0));
  }

  private void stingerFire() {
    for (int i = 0; i < buffer.getLength(); i++) {
      double wave = 0.5 + 0.5 * Math.sin((i * 0.8) - (tick * 0.7));

      Color c;
      if (wave > 0.7) c = Color.kWhite;
      else if (wave > 0.4) c = new Color(1.0, 0.45, 0.0);
      else c = new Color(0.25, 0.08, 0.0);

      buffer.setLED(i, c);
    }
    push();
  }

  private void rainbow(double speed) {
    for (int i = 0; i < buffer.getLength(); i++) {
      int hue = (int) ((i * 180.0 / buffer.getLength()) + (tick * speed)) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    push();
  }

  private void endgamePulse() {
    double t = DriverStation.getMatchTime();
    double speed = (t <= 10) ? 0.45 : 0.20;

    double p = 0.5 + 0.5 * Math.sin(tick * speed);
    solid(new Color(p, p * 0.55, 0.02));
  }

  // =========================================================
  // HELPERS
  // =========================================================

  private void solid(Color c) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, c);
    }
    push();
  }

  private void push() {
    led.setData(buffer);
  }
}
