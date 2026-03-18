// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Creates a new Subsystem. */
public class ShooterSubsystem extends SubsystemBase {

  // Initialize the motor (Kraken direct drive CAN ID 61)
  private final SlotConfiguration slotConfiguration = new SlotConfiguration();
  private static final CANBus kCANBus = new CANBus("DriveCanivore");
  private final TalonFX shooter = new TalonFX(61, kCANBus);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private VelocityVoltage stopRequest = new VelocityVoltage(0).withSlot(0).withVelocity(0);

  public ShooterSubsystem() {
    Slot0Configs speedConfig = new Slot0Configs();
    speedConfig.kS = 0.1;
    speedConfig.kV = 0.3;
    speedConfig.kP = 0.11;
    speedConfig.kI = 0;
    speedConfig.kD = 0;

    shooter.getConfigurator().apply(speedConfig);
  }

  /** Run shooter at percentage speed */
  public void runShooter(double speed) {
    shooter.setControl(dutyCycleRequest.withOutput(speed));

    // create a velocity closed-loop request, voltage output, slot 0 configs

    // set velocity to 45 rps
    shooter.setControl(velocityRequest.withSlot(0).withVelocity(speed));
  }

  /** Stop shooter */
  public void stopShooter() {
    shooter.setControl(dutyCycleRequest.withOutput(0));

    shooter.setControl(stopRequest);
  }

  /** One-shot command: immediately spins shooter at given speed */
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
  public Command stopCommand() {
    return Commands.runOnce(this::stopShooter, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Get current shooter velocity in rotations per second (RPS) */
  public double getVelocityRps() {
    return shooter.getVelocity().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
