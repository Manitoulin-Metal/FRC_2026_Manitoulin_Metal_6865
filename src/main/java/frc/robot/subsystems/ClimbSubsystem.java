package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  // =========================================================
  // STATE MACHINE (SIMPLE + EXPLICIT)
  // =========================================================
  public enum State {
    IDLE,
    UP,
    DOWN,
    HOMING,
    AT_TOP,
    AT_BOTTOM,
    DISABLED
  }

  private State state = State.IDLE;

  // =========================================================
  // HARDWARE
  // =========================================================
  private final SparkFlex motor = new SparkFlex(60, MotorType.kBrushless);
  private final RelativeEncoder encoder;
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Climb.LIMIT_SWITCH_CHANNEL);

  // =========================================================
  // TRACKING
  // =========================================================
  private boolean homed = false;
  private double homingStartTime = -1;

  // =========================================================
  // CONSTRUCTOR
  // =========================================================
  @SuppressWarnings("removal")
  public ClimbSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  // =========================================================
  // SENSOR HELPERS
  // =========================================================

  public boolean limitPressed() {
    return !limitSwitch.get(); // active low
  }

  public boolean isHomed() {
    return homed;
  }

  public State getState() {
    return state;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  // =========================================================
  // LIMIT LOGIC
  // =========================================================

  private boolean atBottom() {
    return limitPressed() || getPosition() <= Constants.Climb.BOTTOM_ENCODER_TOLERANCE_ROTATIONS;
  }

  private boolean atTop() {
    double top = Constants.Climb.upTargetEntry.get();
    return top > 1.0 && getPosition() >= top;
  }

  // =========================================================
  // COMMAND API (CLEAN & DIRECT)
  // =========================================================

  public void moveUp() {
    System.out.println(state);
    if (state == State.DISABLED) return;
    System.out.println("moveUp called in ClimbSubsystem.");
    state = State.UP;
    System.out.println("State set to UP in ClimbSubsystem.");
  }

  public void moveDown() {
    if (state == State.DISABLED) return;

    state = State.DOWN;
  }

  public void stop() {
    if (state != State.DISABLED) {
      state = State.IDLE;
    }
  }

  public void disable() {
    state = State.DISABLED;
    motor.stopMotor();
  }

  public void startHoming() {
    if (state == State.DISABLED) return;

    if (limitPressed()) {
      encoder.setPosition(0);
      homed = true;
      state = State.AT_BOTTOM;
      return;
    }

    homed = false;
    homingStartTime = Timer.getFPGATimestamp();
    state = State.HOMING;
  }

  // =========================================================
  // COMMAND FACTORY
  // =========================================================

  // public Command upCommand() {
  //   // System.out.println("upCommand in ClimbSystem running.");
  //   // System.out.println("Before state:" + state);
  //   this.state = State.UP;
  //   // System.out.println("After state:" + state);
  //   return Commands.none();
  //   // return Commands.runOnce(this::moveUp, this).withName("ClimbUpCommand");
  // }

  public Command upCommand() {
    return Commands.runOnce(() -> this.state = State.UP, this).withName("ClimbUpCommand");
  }

  public Command downCommand() {
    return Commands.startEnd(this::moveDown, this::stop, this);
  }

  public Command homeCommand() {
    return Commands.sequence(
        Commands.runOnce(this::startHoming, this),
        Commands.waitUntil(() -> isHomed()).withTimeout(Constants.Climb.HOMING_TIMEOUT_SECONDS),
        Commands.runOnce(this::stop, this));
  }

  // =========================================================
  // PERIODIC (PURE STATE MACHINE - NO EXTRA LOGIC)
  // =========================================================
  @Override
  public void periodic() {

    double output = 0.0;

    switch (state) {
      case UP:
        if (atTop()) {
          state = State.AT_TOP;
          output = 0.0;
        } else {
          output = Constants.Climb.UP_SPEED;
        }
        break;

      case DOWN:
        if (atBottom()) {
          encoder.setPosition(0);
          homed = true;
          state = State.AT_BOTTOM;
          output = 0.0;
        } else {
          output = Constants.Climb.DOWN_SPEED;
        }
        break;

      case HOMING:
        if (limitPressed()) {
          encoder.setPosition(0);
          homed = true;
          state = State.AT_BOTTOM;
          output = 0.0;

        } else if (Timer.getFPGATimestamp() - homingStartTime
            > Constants.Climb.HOMING_TIMEOUT_SECONDS) {

          state = State.DISABLED;
          output = 0.0;

        } else {
          output = Constants.Climb.HOMING_SPEED;
        }
        break;

      case AT_TOP:
      case AT_BOTTOM:
      case DISABLED:
      case IDLE:
      default:
        output = 0.0;
        break;
    }

    if (Math.abs(output) < 0.001) {
      motor.stopMotor();
    } else {
      motor.set(output);
    }

    // Logger.recordOutput("Climb/State", state.toString());
    // Logger.recordOutput("Climb/Homed", homed);
    // Logger.recordOutput("Climb/Disabled", state == State.DISABLED);

    // Logger.recordOutput("Climb/EncoderPosition", encoder.getPosition());
    Logger.recordOutput("Climb/LimitSwitchPressed", limitPressed());

    // Logger.recordOutput("Climb/MotorOutput", output);

    Logger.recordOutput("Climb/AtTop", atTop());
    Logger.recordOutput("Climb/AtBottom", atBottom());
  }
}
