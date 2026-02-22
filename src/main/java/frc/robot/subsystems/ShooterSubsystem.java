// This is being used by Team 6865, Manitoulin Metal
// This was created by Team 6865, Manitoulin Metal

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Creates a new Subsystem. */
public class ShooterSubsystem extends SubsystemBase {

    // Initialize the motor (Kraken direct drive CAN ID 61)
    private static final CANBus kCANBus = new CANBus("canivore");
    private final TalonFX shooter = new TalonFX(61, kCANBus);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // run shooter at percentage speed
    public void runShooter(double speed) {
        shooter.setControl(dutyCycleRequest.withOutput(speed));
    }

    // run shooter at percentage speed
    public void stopShooter(double speed) {
        shooter.setControl(dutyCycleRequest.withOutput(0));
    }

    /** Returns a command to run the shooter at a given speed once. */
    public Command shootCommand(double speed) {
        // This creates a one-shot command that requires this subsystem
        return Commands.runOnce(() -> runShooter(speed), this);
    }

    /** Returns a command to stop the shooter. */
    public Command stopCommand() {
        return Commands.runOnce(() -> stopShooter(), this);
    }

    // Optional: helper that stops the motor
    public void stopShooter() {
        shooter.setControl(dutyCycleRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
