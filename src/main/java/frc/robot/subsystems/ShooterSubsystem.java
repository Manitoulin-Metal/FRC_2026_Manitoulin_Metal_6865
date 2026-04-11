// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.jni.*;
import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Creates a new Subsystem. */
@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {

  // Initialize the motor (Kraken direct drive CAN ID 61)

  private static final CANBus kCANBus = new CANBus("rio");
  private final TalonFX shooter = new TalonFX(61, kCANBus);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.1);
  private final VelocityVoltage stopRequest = new VelocityVoltage(0);
  private final LoggedNetworkNumber kPEntry = Constants.Shooter.kPEntry;
  private final LoggedNetworkNumber kIEntry = Constants.Shooter.kIEntry;
  private final LoggedNetworkNumber kDEntry = Constants.Shooter.kDEntry;
  private final LoggedNetworkNumber kVEntry = Constants.Shooter.kVEntry;
  private final LoggedNetworkNumber kSEntry = Constants.Shooter.kSEntry;
  private double targetRps = 95.0;
  private double currentRps = 0.0;

  private final StatusSignal<Integer> faultsSignal = shooter.getFaultField();

  public ShooterSubsystem() {
    Slot0Configs speedConfig = new Slot0Configs();
    speedConfig.kS = Constants.Shooter.kS;
    speedConfig.kV = Constants.Shooter.kV;
    speedConfig.kP = Constants.Shooter.kP;
    speedConfig.kI = Constants.Shooter.kI;
    speedConfig.kD = Constants.Shooter.kD;

    shooter.getConfigurator().apply(speedConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, faultsSignal);
  }

  /** Run shooter at velocity RPS */
  public void runShooter(double speedRps) {
    targetRps = speedRps;
    double rampRate = 25.0; // RPS/sec
    currentRps =
        MathUtil.clamp(
            currentRps + Math.copySign(rampRate * (1.0 / 50.0), speedRps - currentRps),
            -Math.abs(speedRps),
            Math.abs(speedRps));
    shooter.setControl(velocityRequest.withVelocity(currentRps));
  }

  /** Stop shooter */
  public void stopShooter() {
    shooter.setControl(stopRequest);
  }

  public void runOpenLoop(double dutyCycle) {
    shooter.setNeutralMode(NeutralModeValue.Coast);
    shooter.setControl(new DutyCycleOut(dutyCycle));
  }

  public Command shootCommand(double speed) {
    return Commands.runOnce(() -> runShooter(speed), this);
  }

  /** Timed shoot: spins shooter at given speed for 15 seconds */
  public Command timedShootCommand(double speed) {
    return Commands.runEnd(
            () -> runShooter(speed), // start shooting
            this::stopShooter, // stop shooting after finished
            this // requires this subsystem
            )
        .withTimeout(15.0);
  }

  /** Stop command: stops shooter instantly */
  /** Returns true if shooter velocity is at target within tolerance */
  public boolean atTarget() {
    return Math.abs(targetRps - getVelocityRps()) <= Constants.SHOOTER_AT_TARGET_TOLERANCE_RPS
        && getVelocityRps() >= Constants.SHOOTER_MIN_RPS;
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stopShooter, this);
  }

  @Override
  public void periodic() {
    // Live PID/FF tuning updates (like IntakeRoller)
    updatePIDIfChanged();

    // Refresh faults signal
    faultsSignal.refresh();

    double velocityRps = getVelocityRps();
    double error = Math.abs(targetRps - velocityRps);
    double motorVoltage = shooter.getMotorVoltage().getValueAsDouble();
    double statorCurrent = shooter.getStatorCurrent().getValueAsDouble();
    double supplyCurrent = shooter.getSupplyCurrent().getValueAsDouble();
    int faultsRaw = faultsSignal.getValue();

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Shooter/VelocityRPS", velocityRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/TargetRPS", targetRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/PIDError", error);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Shooter/MotorVoltage", motorVoltage);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Shooter/StatorCurrent", statorCurrent);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Shooter/SupplyCurrent", supplyCurrent);
    String faultSummary = faultsRaw == 0 ? "OK" : "0x" + Integer.toHexString(faultsRaw);
    if ((faultsRaw & 1) != 0) faultSummary += " SupplyCurrLimit";
    if ((faultsRaw & 2) != 0) faultSummary += " HardwareCurrLimit";
    if ((faultsRaw & 16) != 0) faultSummary += " UnderVoltage";
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
        "Shooter/FaultSummary", faultSummary);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString(
        "Shooter/Faults", Integer.toHexString(faultsRaw));
    Logger.recordOutput("Shooter/VelocityRPS", velocityRps);
    Logger.recordOutput("Shooter/TargetRPS", targetRps);

    Logger.recordOutput("Shooter/PIDError", error);
    Logger.recordOutput("Shooter/MotorVoltage", motorVoltage);
    Logger.recordOutput("Shooter/StatorCurrent", statorCurrent);
    Logger.recordOutput("Shooter/SupplyCurrent", supplyCurrent);
    Logger.recordOutput("Shooter/Faults", Integer.toHexString(faultsRaw));
    Logger.recordOutput("Shooter/FaultSummary", faultSummary);
  }

  private void updatePIDIfChanged() {
    double newKP = kPEntry.get();
    double newKI = kIEntry.get();
    double newKD = kDEntry.get();
    double newKV = kVEntry.get();
    double newKS = kSEntry.get();

    Slot0Configs config = new Slot0Configs();
    config.kP = newKP;
    config.kI = newKI;
    config.kD = newKD;
    config.kV = newKV;
    config.kS = newKS;

    shooter.getConfigurator().apply(config);
  }

  /** Get current shooter velocity in rotations per second (RPS) */
  public double getVelocityRps() {
    return shooter.getVelocity().getValueAsDouble();
  }

  /**
   * Calculate target RPS based on distance to shooting target using interpolation table.
   *
   * @param distanceMeters Distance to AprilTag (25 or 26)
   * @return Interpolated target RPS
   */
  public double calculateTargetRPS(double distanceMeters) {
    double[] distances = Constants.SHOOTER_DISTANCE_BREAKPOINTS_METERS;
    double[] rpsValues = Constants.SHOOTER_TARGET_RPS_BY_DISTANCE;

    // Clamp distance to valid range
    distanceMeters = MathUtil.clamp(distanceMeters, distances[0], distances[distances.length - 1]);

    // Find interval
    int index = 0;
    for (int i = 0; i < distances.length - 1; i++) {
      if (distanceMeters <= distances[i + 1]) {
        index = i;
        break;
      }
    }

    // Linear interpolation
    if (index == distances.length - 1) {
      return rpsValues[index]; // Past last point
    }
    double fraction =
        (distanceMeters - distances[index]) / (distances[index + 1] - distances[index]);
    return MathUtil.interpolate(rpsValues[index], rpsValues[index + 1], fraction);
  }

  /**
   * Get average distance to valid shooting targets (tags 25/26) from Limelight.
   *
   * @param vision VisionSubsystem instance
   * @return Average distToRobot in meters, or -1 if no valid targets
   */
  public double getTargetDistance(Vision vision) {
    var rawFiducials = LimelightHelpers.getRawFiducials("limelight");
    if (rawFiducials.length == 0) {
      return -1.0;
    }

    double totalDist = 0.0;
    int validCount = 0;

    for (var fiducial : rawFiducials) {
      if ((fiducial.id == Constants.SHOOTING_TAG_IDS[0]
              || fiducial.id == Constants.SHOOTING_TAG_IDS[1])
          && fiducial.distToRobot > Constants.MIN_SHOOT_DISTANCE_METERS
          && fiducial.distToRobot <= Constants.MAX_SHOOT_DISTANCE_METERS) {
        totalDist += fiducial.distToRobot;
        validCount++;
      }
    }

    return validCount > 0 ? totalDist / validCount : -1.0;
  }

  /**
   * Run shooter using vision distance. Automatically calculates and ramps to target RPS.
   *
   * @param vision VisionSubsystem
   */
  public void runVisionShooter(Vision vision) {
    double distance = getTargetDistance(vision);
    if (distance > 0) {
      double targetRps = calculateTargetRPS(distance);
      runShooter(targetRps);
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
          "Shooter/TargetDistanceM", distance);
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
          "Shooter/VisionTargetRPS", targetRps);
      Logger.recordOutput("Shooter/TargetDistanceM", distance);
      Logger.recordOutput("Shooter/VisionTargetRPS", targetRps);
    } else {
      stopShooter();
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/TargetDistanceM", -1);
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Shooter/VisionTargetRPS", 0);
      Logger.recordOutput("Shooter/TargetDistanceM", -1.0);
      Logger.recordOutput("Shooter/VisionTargetRPS", 0.0);
    }
  }

  /** Open-loop test at 50% output */
  public Command openLoopTestCommand() {
    return Commands.runEnd(() -> runOpenLoop(0.5), this::stopShooter, this).withTimeout(5);
  }

  /** Clear sticky faults */
  public Command clearFaultsCommand() {
    return Commands.runOnce(() -> shooter.clearStickyFaults(), this);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
