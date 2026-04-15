package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@SuppressWarnings("deprecated")
public class IntakeRollerSubsystem extends SubsystemBase {

  private final SparkFlex intakeRoller = new SparkFlex(58, MotorType.kBrushless);
  private final SparkClosedLoopController velocityController;

  // ===== Live tuning =====
  private final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Intake/kP", 0.00025);
  private final LoggedNetworkNumber kFF = new LoggedNetworkNumber("/Intake/kFF", 0.00017);
  private final LoggedNetworkNumber targetRPM = new LoggedNetworkNumber("/Intake/TargetRPM", -3500);

  private double lastKP = -1;
  private double lastKFF = -1;

  public enum Mode {
    IDLE,
    INTAKE
  }

  private Mode currentMode = Mode.IDLE;

  private static final double IDLE_RPM = 0;
  private static final double INTAKE_RPM = 4500;

  // ===== Driver control protection =====
  private boolean manualIntake = false;
  private final Timer intakeStartTimer = new Timer();

  // ===== Tracking =====
  private int ballCount = 0;
  private final Timer detectionTimer = new Timer();
  private boolean pieceLatched = false;

  public IntakeRollerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.p(0.00025).i(0.0).d(0.0).velocityFF(0.00017).outputRange(-1, 1);

    intakeRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    velocityController = intakeRoller.getClosedLoopController();

    detectionTimer.start();
    intakeStartTimer.start();
  }

  // ================= COMMAND =================

  public Command intakeToggleCommand() {
    return startEnd(
        () -> {
          manualIntake = true;
          intakeStartTimer.reset();
          intakeStartTimer.start();
          setMode(Mode.INTAKE);
        },
        () -> {
          manualIntake = false;
          setMode(Mode.IDLE);
        });
  }

  // ================= CONTROL =================

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  public void setVelocity(double rpm) {
    velocityController.setSetpoint(rpm, ControlType.kVelocity);
  }

  // ================= PID UPDATE =================

  private void updatePIDIfChanged() {
    double newKP = kP.get();
    double newKFF = kFF.get();

    if (newKP != lastKP || newKFF != lastKFF) {
      SparkFlexConfig config = new SparkFlexConfig();

      config.closedLoop.p(newKP).i(0.0).d(0.0).velocityFF(newKFF).outputRange(-1, 1);

      intakeRoller.configure(
          config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      lastKP = newKP;
      lastKFF = newKFF;
    }
  }

  // ================= SENSOR HELPERS =================

  public double getVelocity() {
    return intakeRoller.getEncoder().getVelocity();
  }

  public double getCurrent() {
    return intakeRoller.getOutputCurrent();
  }

  public boolean gamePieceDetected(double target) {
    return getCurrent() > 35 && getVelocity() < target * 0.6;
  }

  // ================= PERIODIC =================

  @Override
  public void periodic() {
    updatePIDIfChanged();

    double velocity = getVelocity();
    double current = getCurrent();
    double target = targetRPM.get();

    // Prevent false detection during spin-up
    boolean allowDetection = intakeStartTimer.get() > 0.5;
    boolean detected = allowDetection && gamePieceDetected(target);

    switch (currentMode) {
      case IDLE -> setVelocity(IDLE_RPM);

      case INTAKE -> {
        setVelocity(INTAKE_RPM);

        // Only track pieces, DO NOT change mode
        if (detected && !pieceLatched && detectionTimer.get() > 0.25) {
          ballCount++;
          pieceLatched = true;
          detectionTimer.reset();
        }

        if (!detected) pieceLatched = false;
      }
    }

    // ===== Logging =====
    Logger.recordOutput("Intake/Mode", currentMode.toString());
    Logger.recordOutput("Intake/RPM", velocity);
    Logger.recordOutput("Intake/TargetRPM", target);
    Logger.recordOutput("Intake/Current", current);
    Logger.recordOutput("Intake/BallCount", ballCount);
    Logger.recordOutput("Intake/Detected", detected);
  }
}
