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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  public enum ClimbState {
    DISABLED,
    IDLE,
    UP,
    DOWN,
    HOMING,
    AT_TOP,
    AT_BOTTOM
  }

  private final SparkFlex climbMotor = new SparkFlex(60, MotorType.kBrushless);
  private final RelativeEncoder encoder;
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Climb.LIMIT_SWITCH_CHANNEL);
  private boolean manualOverride = false;

  private ClimbState state = ClimbState.IDLE;
  private boolean homed = false;
  private boolean upTargetReached = false;
  private double homingStartTimestamp = -1.0;

  public void setManualOverride(boolean override) {
    manualOverride = override;
  }

  @SuppressWarnings("removal")
  public ClimbSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = climbMotor.getEncoder();
  }

  // ========================= MOTOR CONTROL =========================

  private boolean isMotionAllowed() {
    return !manualOverride;
  }

  public boolean isLimitSwitchPressed() {
    return !limitSwitch.get(); // active low
  }

  public ClimbState getState() {
    return state;
  }

  public boolean isHomed() {
    return homed;
  }

  public boolean isDisabled() {
    return state == ClimbState.DISABLED;
  }

  public void disable() {
    state = ClimbState.DISABLED;
    homingStartTimestamp = -1.0;
  }

  public void moveUp() {
    if (isDisabled() || !isMotionAllowed()) {
      return;
    }

    if (isTopLimitReached()) {
      upTargetReached = true;
      state = ClimbState.AT_TOP;
      return;
    }

    state = ClimbState.UP;
  }

  public void moveDown() {
    if (isDisabled() || !isMotionAllowed()) {
      return;
    }

    if (isBottomLimitReached()) {
      state = ClimbState.AT_BOTTOM;
      return;
    }

    state = ClimbState.DOWN;
  }

  public void stop() {
    if (!isDisabled()) {
      state = ClimbState.IDLE;
    }
  }

  public void startHoming() {
    if (isDisabled() || !isMotionAllowed()) {
      return;
    }

    if (isLimitSwitchPressed()) {
      encoder.setPosition(0.0);
      homed = true;
      state = ClimbState.AT_BOTTOM;
      homingStartTimestamp = -1.0;
      return;
    }

    homed = false;
    state = ClimbState.HOMING;
    homingStartTimestamp = Timer.getFPGATimestamp();
  }

  // ========================= COMMANDS =========================

  public Command climbCommand(double speed) {
    return Commands.runEnd(
        () -> {
          if (isDisabled() || !isMotionAllowed()) {
            return;
          }

          if (speed > 0.0) {
            moveUp();
          } else if (speed < 0.0) {
            moveDown();
          } else {
            state = ClimbState.IDLE;
          }
        },
        this::stop,
        this);
  }

  public Command homeCommand() {
    return Commands.sequence(
            Commands.runOnce(this::startHoming, this),
            Commands.waitUntil(() -> isHomed() || isDisabled())
                .withTimeout(Constants.Climb.HOMING_TIMEOUT_SECONDS))
        .andThen(
            Commands.runOnce(
                () -> {
                  if (!isHomed()) {
                    disable();
                  }
                },
                this));
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public boolean isAtUpTarget() {
    return upTargetReached;
  }

  private boolean isTopLimitReached() {
    double upTarget = Constants.Climb.upTargetEntry.get();
    return upTarget > 1.0 && encoder.getPosition() >= upTarget;
  }

  private boolean isBottomLimitReached() {
    return isLimitSwitchPressed()
        || encoder.getPosition() <= Constants.Climb.BOTTOM_ENCODER_TOLERANCE_ROTATIONS;
  }

  @Override
  public void periodic() {
    if (manualOverride) {
      climbMotor.stopMotor();
      state = ClimbState.IDLE; // prevents "ghost climbing"
      return;
    }
    boolean pressed = isLimitSwitchPressed();
    double output = 0.0;
    double climbPosition = encoder.getPosition();
    double upTarget = Constants.Climb.upTargetEntry.get();

    switch (state) {
      case UP:
        if (upTarget > 1.0 && climbPosition >= upTarget) {
          output = 0.0;
          upTargetReached = true;
          state = ClimbState.AT_TOP;
        } else {
          output = Constants.Climb.UP_SPEED;
        }
        break;

      case DOWN:
        if (pressed || climbPosition <= Constants.Climb.BOTTOM_ENCODER_TOLERANCE_ROTATIONS) {
          output = 0.0;
          encoder.setPosition(0.0);
          state = ClimbState.AT_BOTTOM;
          if (pressed) {
            homed = true;
          }
        } else {
          output = Constants.Climb.DOWN_SPEED;
        }
        break;

      case HOMING:
        if (pressed) {
          output = 0.0;
          encoder.setPosition(0.0);
          state = ClimbState.AT_BOTTOM;
          homed = true;
          homingStartTimestamp = -1.0;
        } else if (homingStartTimestamp > 0.0
            && (Timer.getFPGATimestamp() - homingStartTimestamp)
                >= Constants.Climb.HOMING_TIMEOUT_SECONDS) {
          output = 0.0;
          state = ClimbState.DISABLED;
          homingStartTimestamp = -1.0;
        } else {
          output = Constants.Climb.HOMING_SPEED;
        }
        break;

      case DISABLED:
        output = 0.0;
        break;

      case AT_TOP:
        output = 0.0;
        break;

      case AT_BOTTOM:
        output = 0.0;
        break;

      case IDLE:
      default:
        output = 0.0;
        break;
    }

    if (output == 0.0) {
      climbMotor.stopMotor();
    } else {
      climbMotor.set(output);
    }

    // Smart Dashboard updates for tuning and debugging - commented out some to avoid
    // loop overun and additional logging for AdvantageKit Logger data analysis

    SmartDashboard.putString("Climb/State", state.name());
    SmartDashboard.putNumber("Climb/Position", climbPosition);
    SmartDashboard.putBoolean("Climb/LimitSwitch", pressed);
    SmartDashboard.putBoolean("Climb/Homed", homed);
    SmartDashboard.putBoolean("Climb/ManualOverride", manualOverride);

    Logger.recordOutput("Climb/State", state.name());
    Logger.recordOutput("Climb/Homed", homed);
    Logger.recordOutput("Climb/Disabled", isDisabled());
    Logger.recordOutput("Climb/LimitSwitchPressed", pressed);
    Logger.recordOutput("Climb/SpeedCommand", output);
    Logger.recordOutput("Climb/EncoderPosition", climbPosition);
    Logger.recordOutput("Climb/UpTargetRotations", upTarget);
    Logger.recordOutput("Climb/TopLimitReached", upTarget > 1.0 && climbPosition >= upTarget);
    Logger.recordOutput(
        "Climb/BottomLimitReached",
        pressed || climbPosition <= Constants.Climb.BOTTOM_ENCODER_TOLERANCE_ROTATIONS);

    // SmartDashboard.putBoolean("Climb/Disabled", isDisabled());
    // SmartDashboard.putNumber("Climb/SpeedCommand", output);
    // SmartDashboard.putNumber("Climb/EncoderPosition", climbPosition);
    // SmartDashboard.putNumber("Climb/UpTargetRotations", upTarget);
    // SmartDashboard.putBoolean("Climb/TopLimitReached", upTarget > 1.0 &&
    // climbPosition >=
    // upTarget);
    // SmartDashboard.putBoolean(
    // "Climb/BottomLimitReached",
    // pressed || climbPosition <=
    // Constants.Climb.BOTTOM_ENCODER_TOLERANCE_ROTATIONS);
  }
}
