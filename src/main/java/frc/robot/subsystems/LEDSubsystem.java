package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  // =========================================================
  // LED CONFIG
  // =========================================================

  private static final int PWM_PORT = 9;
  private static final int LED_LENGTH = 100;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  // =========================================================
  // ANIMATION STATE
  // =========================================================

  private int tick = 0;

  // =========================================================
  // LED STATES
  // =========================================================

  public enum LEDState {
    DISABLED(10),
    ENABLED(20),
    INTAKING(60),
    VISION_LOCK(70),
    SHOOTER_READY(75),
    SHOOTING(80),
    CLIMBING_UP(90),
    CLIMBING_DOWN(90),
    ENDGAME(100);

    public final int priority;

    LEDState(int priority) {
      this.priority = priority;
    }
  }

  private LEDState currentState = LEDState.DISABLED;

  // =========================================================
  // CONSTRUCTOR
  // =========================================================

  public LEDSubsystem() {

    led = new AddressableLED(PWM_PORT);
    buffer = new AddressableLEDBuffer(LED_LENGTH);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  // =========================================================
  // STATE CONTROL
  // =========================================================

  public void requestState(LEDState newState) {

    if (newState.priority >= currentState.priority) {
      currentState = newState;
    }
  }

  public void clearState(LEDState state) {

    if (currentState == state) {

      if (DriverStation.isDisabled()) {
        currentState = LEDState.DISABLED;
      } else {
        currentState = LEDState.ENABLED;
      }
    }
  }

  // =========================================================
  // PERIODIC
  // =========================================================

  @Override
  public void periodic() {

    tick++;

    // =======================================================
    // BASE STATE MANAGEMENT
    // =======================================================

    if (DriverStation.isDisabled()) {

      currentState = LEDState.DISABLED;

    } else if (currentState == LEDState.DISABLED) {

      currentState = LEDState.ENABLED;
    }

    // =======================================================
    // ENDGAME AUTO OVERRIDE
    // =======================================================

    if (DriverStation.isTeleopEnabled()) {

      double matchTime = DriverStation.getMatchTime();

      // Automatically override during endgame
      if (matchTime > 0 && matchTime <= 20) {

        currentState = LEDState.ENDGAME;
      }
    }

    // =======================================================
    // STATE MACHINE
    // =======================================================

    switch (currentState) {
      case DISABLED:
        beeIdlePattern();
        break;

      case ENABLED:
        queenBeePulse();
        break;

      case INTAKING:
        nectarFlow();
        break;

      case VISION_LOCK:
        visionLock();
        break;

      case SHOOTER_READY:
        shooterReady();
        break;

      case SHOOTING:
        stingerFire();
        break;

      case CLIMBING_UP:
        climbingUpPattern();
        break;

      case CLIMBING_DOWN:
        climbingDownPattern();
        break;

      case ENDGAME:
        endgamePulse();
        break;
    }
  }

  // =========================================================
  // DISABLED - HIVE IDLE
  // =========================================================

  private void beeIdlePattern() {

    for (int i = 0; i < buffer.getLength(); i++) {

      double wave = 0.5 + 0.5 * Math.sin((i * 0.18) + (tick * 0.03));

      Color color;

      if (wave > 0.72) {

        // warm honey gold
        color = new Color(0.95, 0.72, 0.18);

      } else if (wave > 0.48) {

        // amber
        color = new Color(0.75, 0.45, 0.08);

      } else if (wave > 0.28) {

        // dim gold
        color = new Color(0.38, 0.28, 0.05);

      } else {

        // soft shadow
        color = new Color(0.02, 0.015, 0.005);
      }

      buffer.setLED(i, color);
    }

    push();
  }

  // =========================================================
  // ENABLED - QUEEN BEE
  // =========================================================

  private void queenBeePulse() {

    double pulse = 0.35 + 0.15 * Math.sin(tick * 0.04);

    Color color = new Color(pulse, pulse * 0.75, 0.08);

    solid(color);
  }

  // =========================================================
  // INTAKING - NECTAR FLOW
  // =========================================================

  private void nectarFlow() {

    for (int i = 0; i < buffer.getLength(); i++) {

      double wave = 0.5 + 0.5 * Math.sin((i * 0.45) - (tick * 0.25));

      Color color = new Color(0.0, wave * 0.8, 0.05);

      buffer.setLED(i, color);
    }

    push();
  }

  // =========================================================
  // VISION LOCK
  // =========================================================

  private void visionLock() {

    double pulse = 0.5 + 0.5 * Math.sin(tick * 0.12);

    solid(new Color(pulse, pulse * 0.8, 0.1));
  }

  // =========================================================
  // SHOOTER READY
  // =========================================================

  private void shooterReady() {

    double pulse = 0.4 + 0.4 * Math.sin(tick * 0.18);

    solid(new Color(pulse, pulse * 0.45, 0.0));
  }

  // =========================================================
  // SHOOTING - STINGER FIRE
  // =========================================================

  private void stingerFire() {

    for (int i = 0; i < buffer.getLength(); i++) {

      double wave = 0.5 + 0.5 * Math.sin((i * 0.8) - (tick * 0.7));

      Color color;

      if (wave > 0.7) {

        color = Color.kWhite;

      } else if (wave > 0.4) {

        color = new Color(1.0, 0.45, 0.0);

      } else {

        color = new Color(0.25, 0.08, 0.0);
      }

      buffer.setLED(i, color);
    }

    push();
  }

  // =========================================================
  // CLIMBING - Up and Down can share the same pattern but with different speeds
  // =========================================================

  private void climbingUpPattern() {
    rainbowChase(0.75); // slow
  }

  private void climbingDownPattern() {
    rainbowChase(3.0); // fast
  }

  // =========================================================
  // ENDGAME
  // =========================================================

  private void endgamePulse() {

    double matchTime = DriverStation.getMatchTime();

    double speed;

    // Final 10 seconds = faster pulse
    if (matchTime <= 10) {

      speed = 0.45;

    } else {

      speed = 0.20;
    }

    double pulse = 0.5 + 0.5 * Math.sin(tick * speed);

    Color color = new Color(pulse, pulse * 0.55, 0.02);

    solid(color);
  }

  // =========================================================
  // HELPERS
  // =========================================================

  private void solid(Color color) {

    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }

    push();
  }

  private void push() {
    led.setData(buffer);
  }

  private void rainbowChase(double speed) {

    for (int i = 0; i < buffer.getLength(); i++) {

      int hue = (int) ((i * 180.0 / buffer.getLength()) + (tick * speed)) % 180;

      buffer.setHSV(i, hue, 255, 128);
    }

    push();
  }
}
