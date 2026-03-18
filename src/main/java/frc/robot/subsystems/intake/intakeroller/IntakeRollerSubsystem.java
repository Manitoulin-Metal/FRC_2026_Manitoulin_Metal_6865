// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems.intake.intakeroller;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  // Motor initialized in constructor

  // Initialize the motor (Flex/MAX are setup the same way)
  private final SparkFlex intakeRoller = new SparkFlex(58, MotorType.kBrushless);
  private final SparkClosedLoopController velocityController;

// ================= LIVE TUNING ================= \\
  private final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Intake/kP", 0.00025);
  private final LoggedNetworkNumber kFF = new LoggedNetworkNumber("/Intake/kFF", 0.00017);
  private final LoggedNetworkNumber targetRPM = new LoggedNetworkNumber("/Intake/TargetRPM", 3500);

  private double lastKP = -1;
  private double lastKFF = -1;

  public enum Mode {
    IDLE, INTAKE, HOLD, UNJAM
  }

  private Mode currentMode = Mode.IDLE;

  // Fallback RPMs (used if network tables values are not set)
  private static final double IDLE_RPM = 1200;
  private static final double HOLD_RPM = 500;
  private static final double UNJAM_RPM = -2000;

  // Tracking
  private int ballCount = 0;
  private int jamCount = 0;

  private final Timer detectionTimer = new Timer();
  private boolean pieceLatched = false;

  private final Timer jamTimer = new Timer();



  /** Creates a new Subsystem. */
  public IntakeRollerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop
        .p(0.00025)
        .i(0.0)
        .d(0.0)
        .velocityFF(0.00017)
        .outputRange(-1,1);

    intakeRoller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    velocityController = intakeRoller.getClosedLoopController();

    detectionTimer.start();
    jamTimer.start();
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */

  // ----------------- COMMANDS ----------------- \\
  public Command intakeCommand() {
    return run(
      () ->
      setMode(Mode.INTAKE));
  }  

  public Command idleCommand() {
    return run(
      () ->
      setMode(Mode.IDLE));
  }

  public void restBallCount() {
    ballCount = 0;
    jamCount = 0;
  }

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  public void setVelocity(double rpm) {
    velocityController.setSetpoint(rpm, ControlType.kVelocity);
  }

  // --------- Live PID Tuning Updates --------- \\
  private void updatePIDIfChanged() {
    double newKP = kP.get();
    double newKFF = kFF.get();

    if (newKP != lastKP || newKFF != lastKFF) {
      SparkFlexConfig config = new SparkFlexConfig();

      config.closedLoop
      .p(newKP)
      .i(0.0)
      .d(0.0)
      .velocityFF(newKFF)
      .outputRange(-1,1);

      intakeRoller.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

      lastKP = newKP;
      lastKFF = newKFF;
    }
  }

  // --------- SENSOR METHODS --------- \\
  public double getVelocity() {
    return intakeRoller.getEncoder().getVelocity();
  }

  public double getCurrent() {
    return intakeRoller.getOutputCurrent();
  }

  public boolean gamePieceDetected(double target) {
    return getCurrent() > 30 || getVelocity() < target * 0.75;
  }

  public boolean jamDetected(double target) {
    return getCurrent() > 40 && getVelocity() < target * 0.4;
  }

  // ---------- PERIODIC ---------- \\
  public void periodic() {
    // Apply live PID updates if changed
    updatePIDIfChanged();

    double velocity = getVelocity();
    double current = getCurrent();

    // Safe way to get NetworkTable Values (To prevent Crashes)
    double target = targetRPM.get();

    boolean detected = gamePieceDetected(target);

    switch (currentMode) {

      case IDLE -> setVelocity(IDLE_RPM);

      case INTAKE -> {
        setVelocity(target);

        if (jamDetected(target)) {
          currentMode = Mode.UNJAM;
          jamTimer.reset();
          jamCount++;
        }

        if (detected && !pieceLatched && detectionTimer.get() > 0.25) {
          ballCount++;
          pieceLatched = true;
          detectionTimer.reset();
          currentMode = Mode.HOLD;
        }

        if (!detected)
          pieceLatched = false;

    }
      case HOLD ->
        setVelocity(HOLD_RPM);

      case UNJAM -> {
        setVelocity(UNJAM_RPM);
        if (jamTimer.get() > 0.25) {
          currentMode = Mode.INTAKE;
        }
      }
    }

    // ----- LOGGING ----- \\
    Logger.recordOutput("Intake/Mode", currentMode.toString());
    Logger.recordOutput("Intake/RPM", velocity);
    Logger.recordOutput("Intake/TargetRPM", target);
    Logger.recordOutput("Intake/Current", current);
    Logger.recordOutput("Intake/BallCount", ballCount);
    Logger.recordOutput("Intake/JamCount", jamCount);
  }
}