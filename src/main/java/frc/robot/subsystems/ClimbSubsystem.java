package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  public enum ClimbState {
    IDLE,
    UP,
    DOWN,
    HOMING
  }

  private final SparkFlex climbMotor = new SparkFlex(60, MotorType.kBrushless);
  private final RelativeEncoder encoder;
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Climb.LIMIT_SWITCH_CHANNEL);

  private static final double UP_SPEED = 0.75;
  private static final double DOWN_SPEED = -0.75;
  private static final double HOMING_SPEED = -0.35;

  private ClimbState state = ClimbState.IDLE;
  private boolean homed = false;
  private boolean upTargetReached = false;

  public ClimbSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = climbMotor.getEncoder();
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

  public void moveUp() {
    upTargetReached = false;
    state = ClimbState.UP;
  }

  public void moveDown() {
    state = ClimbState.DOWN;
  }

  public void stop() {
    state = ClimbState.IDLE;
  }

  public void startHoming() {
    if (isLimitSwitchPressed()) {
      encoder.setPosition(0.0);
      homed = true;
      state = ClimbState.IDLE;
      return;
    }

    homed = false;
    state = ClimbState.HOMING;
  }

  // ========================= COMMANDS =========================

  public Command climbCommand(double speed) {
    return Commands.runEnd(
        () -> {
          if (speed > 0.0) {
            state = ClimbState.UP;
          } else if (speed < 0.0) {
            state = ClimbState.DOWN;
          } else {
            state = ClimbState.IDLE;
          }
        },
        () -> state = ClimbState.IDLE,
        this);
  }

  public Command homeCommand() {
    return Commands.runOnce(this::startHoming, this);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  /**
   * Returns true only after periodic() has confirmed the climber physically
   * reached
   * the UP target. Cleared each time moveUp() is called.
   */
  public boolean isAtUpTarget() {
    return upTargetReached;
  }

  @Override
  public void periodic() {
    boolean pressed = isLimitSwitchPressed();
    double output = 0.0;
    double climbPosition = encoder.getPosition();
    double upTarget = Constants.Climb.upTargetEntry.get();

    switch (state) {
      case UP:
        if (upTarget > 1.0 && climbPosition >= upTarget) {
          output = 0.0;
          upTargetReached = true;
          state = ClimbState.IDLE;
        } else {
          output = UP_SPEED;
        }
        break;

      case DOWN:
        if (pressed) {
          output = 0.0;
          encoder.setPosition(0.0);
          state = ClimbState.IDLE;
          homed = true;
        } else {
          output = DOWN_SPEED;
        }
        break;

      case HOMING:
        if (pressed) {
          output = 0.0;
          encoder.setPosition(0.0);
          state = ClimbState.IDLE;
          homed = true;
        } else {
          output = HOMING_SPEED;
        }
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

    SmartDashboard.putString("Climb/State", state.name());
    SmartDashboard.putBoolean("Climb/Homed", homed);
    SmartDashboard.putBoolean("Climb/LimitSwitchPressed", pressed);
    SmartDashboard.putNumber("Climb/SpeedCommand", output);
    SmartDashboard.putNumber("Climb/EncoderPosition", climbPosition);
    SmartDashboard.putNumber("Climb/UpTargetRotations", upTarget);
  }
}
