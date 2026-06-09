package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

  private final SparkFlex intakeRoller =
      new SparkFlex(Constants.Intake.MOTOR_ID, MotorType.kBrushless);

  private final SparkClosedLoopController velocityController;
  private final LEDSubsystem led;

  public enum Mode {
    IDLE,
    INTAKE,
    REVERSE,
    UNJAM_REVERSE,
    UNJAM_FORWARD
  }

  private Mode currentMode = Mode.IDLE;

  private final Timer intakeStartTimer = new Timer();
  private final Timer jamTimer = new Timer();
  private final Timer countTimer = new Timer();

  private boolean pieceLatched = false;

  public IntakeRollerSubsystem(LEDSubsystem led) {
    this.led = led;

    SparkFlexConfig config = new SparkFlexConfig();

    config
        .closedLoop
        .p(Constants.Intake.KP)
        .i(0.0)
        .d(0.0)
        .velocityFF(Constants.Intake.KFF)
        .outputRange(-1, 1);

    intakeRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    velocityController = intakeRoller.getClosedLoopController();

    intakeStartTimer.start();
    jamTimer.start();
    countTimer.start();
  }

  // ================= COMMANDS =================

  public Command intakeToggleCommand() {
    return startEnd(
        () -> {
          intakeStartTimer.restart();
          jamTimer.restart();
          setMode(Mode.INTAKE);
        },
        () -> setMode(Mode.IDLE));
  }

  public Command reverseCommand() {
    return startEnd(() -> setMode(Mode.REVERSE), () -> setMode(Mode.IDLE));
  }

  public Command stopCommand() {
    return runOnce(() -> setMode(Mode.IDLE));
  }

  public Command collectFuelCommand() {
    return intakeToggleCommand();
  }

  // ================= HELPERS =================

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  public void setVelocity(double rpm) {
    velocityController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public double getVelocity() {
    return intakeRoller.getEncoder().getVelocity();
  }

  public double getCurrent() {
    return intakeRoller.getOutputCurrent();
  }

  public boolean jamDetected() {
    return getCurrent() > Constants.Intake.JAM_CURRENT_AMPS
        && Math.abs(getVelocity())
            < Math.abs(Constants.Intake.INTAKE_RPM) * Constants.Intake.JAM_VELOCITY_RATIO;
  }

  // ================= PERIODIC =================

  @Override
  public void periodic() {

    boolean startupDone = intakeStartTimer.get() > Constants.Intake.STARTUP_IGNORE_TIME;

    switch (currentMode) {
      case IDLE -> {
        setVelocity(Constants.Intake.IDLE_RPM);
        led.clearState(LEDSubsystem.LEDState.INTAKING);
      }

      case REVERSE -> {
        setVelocity(Constants.Intake.REVERSE_RPM);
        led.clearState(LEDSubsystem.LEDState.INTAKING);
      }

      case INTAKE -> {
        setVelocity(Constants.Intake.INTAKE_RPM);

        // 🐝 LED ONLY ACTIVE DURING INTAKE
        led.requestState(LEDSubsystem.LEDState.INTAKING);

        // -------- JAM DETECTION (FIXED TIMER LOGIC) --------
        if (startupDone && jamDetected()) {

          if (jamTimer.get() > Constants.Intake.JAM_DETECT_TIME) {

            currentMode = Mode.UNJAM_REVERSE;
            jamTimer.restart();
          }

        } else {
          jamTimer.restart(); // ✅ FIX: was reset()
        }

        if (jamDetected() && !pieceLatched && countTimer.get() > Constants.Intake.COUNT_DELAY) {

          pieceLatched = true;
          countTimer.restart();
        }

        if (!jamDetected()) {
          pieceLatched = false;
        }
      }

      case UNJAM_REVERSE -> {
        setVelocity(Constants.Intake.REVERSE_RPM);

        // keep LED intake state so driver sees "still working"
        led.requestState(LEDSubsystem.LEDState.INTAKING);

        if (jamTimer.get() > Constants.Intake.UNJAM_REVERSE_TIME) {

          currentMode = Mode.UNJAM_FORWARD;
          jamTimer.restart();
        }
      }

      case UNJAM_FORWARD -> {
        setVelocity(Constants.Intake.INTAKE_RPM);

        led.requestState(LEDSubsystem.LEDState.INTAKING);

        if (jamTimer.get() > Constants.Intake.UNJAM_FORWARD_TIME) {

          currentMode = Mode.INTAKE;
          jamTimer.restart();
        }
      }
    }
  }
}
