package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * IntakeRollerSubsystem
 *
 * <p>Responsibilities: - Runs intake roller in IDLE / INTAKE / REVERSE - Detects stalls using
 * current + velocity logic - Recovers automatically using short reverse burst - Prevents
 * oscillation with cooldown + spin-up delay
 *
 * <p>Does NOT: - Count game pieces - Make scoring decisions
 */
public class IntakeRollerSubsystem extends SubsystemBase {

  private final SparkFlex intakeRoller = new SparkFlex(58, MotorType.kBrushless);
  private final SparkClosedLoopController velocityController;

  // ============================================================
  // MODES
  // ============================================================
  public enum Mode {
    IDLE,
    INTAKE,
    REVERSE
  }

  private Mode currentMode = Mode.IDLE;

  // ============================================================
  // ANTI-JAM STATE MACHINE
  // ============================================================
  private boolean inRecovery = false;
  private Mode previousMode = Mode.IDLE;

  private final Timer recoveryTimer = new Timer();
  private final Timer spinUpTimer = new Timer();
  private final Timer cooldownTimer = new Timer();

  // ============================================================
  // CONSTRUCTOR
  // ============================================================
  @SuppressWarnings("removal")
  public IntakeRollerSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.p(0.00025).i(0.0).d(0.0).velocityFF(0.00017).outputRange(-1, 1);

    intakeRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    velocityController = intakeRoller.getClosedLoopController();

    spinUpTimer.start();
  }

  // ============================================================
  // COMMANDS
  // ============================================================

  public Command intakeToggleCommand() {
    return startEnd(
        () -> {
          setMode(Mode.INTAKE);
          spinUpTimer.reset();
          spinUpTimer.start();
        },
        () -> setMode(Mode.IDLE));
  }

  public Command reverseCommand() {
    return startEnd(() -> setMode(Mode.REVERSE), () -> setMode(Mode.IDLE));
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> setMode(Mode.IDLE));
  }

  // ============================================================
  // MODE CONTROL
  // ============================================================

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  private void setVelocity(double rpm) {
    velocityController.setSetpoint(rpm, ControlType.kVelocity);
  }

  // ============================================================
  // JAM DETECTION
  // ============================================================

  /**
   * A jam is detected when: - motor is drawing high current - but velocity is abnormally low - and
   * intake has finished spin-up
   */
  private boolean isJammed(double current, double velocity) {

    boolean spunUp = spinUpTimer.get() > Constants.Intake.SPINUP_TIME_SEC;
    if (!spunUp) return false;

    boolean highCurrent = current > Constants.Intake.STALL_CURRENT_AMPS;
    boolean lowVelocity = Math.abs(velocity) < Constants.Intake.STALL_VELOCITY_THRESHOLD;

    return highCurrent && lowVelocity;
  }

  // ============================================================
  // RECOVERY FLOW
  // ============================================================

  private void startRecovery() {
    previousMode = currentMode;
    inRecovery = true;

    recoveryTimer.reset();
    recoveryTimer.start();
  }

  private void endRecovery() {
    inRecovery = false;

    recoveryTimer.stop();
    cooldownTimer.reset();
    cooldownTimer.start();

    currentMode = previousMode;
  }

  // ============================================================
  // PERIODIC LOOP
  // ============================================================

  @Override
  public void periodic() {

    double velocity = getVelocity();
    double current = getCurrent();

    // ============================================================
    // 1. RECOVERY MODE (HIGHEST PRIORITY)
    // ============================================================
    if (inRecovery) {

      setVelocity(Constants.Intake.REVERSE_RPM);

      if (recoveryTimer.get() > Constants.Intake.RECOVERY_TIME_SEC) {
        endRecovery();
      }

      Logger.recordOutput("Intake/Mode", "RECOVERY");
      Logger.recordOutput("Intake/InRecovery", true);
      Logger.recordOutput("Intake/Current", current);
      return;
    }

    // ============================================================
    // 2. COOLDOWN (PREVENT JAM OSCILLATION)
    // ============================================================
    if (cooldownTimer.get() < Constants.Intake.COOLDOWN_TIME_SEC) {
      setVelocity(Constants.Intake.IDLE_RPM);

      Logger.recordOutput("Intake/Mode", "COOLDOWN");
      return;
    }

    // ============================================================
    // 3. NORMAL OPERATION
    // ============================================================
    switch (currentMode) {
      case IDLE -> setVelocity(Constants.Intake.IDLE_RPM);

      case INTAKE -> setVelocity(Constants.Intake.INTAKE_RPM);

      case REVERSE -> setVelocity(Constants.Intake.REVERSE_RPM);
    }

    // ============================================================
    // 4. JAM DETECTION (ONLY DURING INTAKE)
    // ============================================================
    if (currentMode == Mode.INTAKE && isJammed(current, velocity)) {
      startRecovery();
    }

    // ============================================================
    // 5. LOGGING
    // ============================================================
    Logger.recordOutput("Intake/Mode", currentMode.toString());
    Logger.recordOutput("Intake/RPM", velocity);
    Logger.recordOutput("Intake/Current", current);
    Logger.recordOutput("Intake/InRecovery", inRecovery);
  }

  // ============================================================
  // GETTERS
  // ============================================================

  public Mode getMode() {
    return currentMode;
  }

  public double getVelocity() {
    return intakeRoller.getEncoder().getVelocity();
  }

  public double getCurrent() {
    return intakeRoller.getOutputCurrent();
  }
}
