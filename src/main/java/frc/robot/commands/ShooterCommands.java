package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WhipSubsystem;

public final class ShooterCommands {

    private ShooterCommands() {
    }

    /** Timed shoot sequence: run shooter and kicker in parallel, then stop both. */
    public static Command timedShoot(
            ShooterSubsystem shooter, KickerSubsystem kicker, double shooterRps, double timeoutSeconds) {
        return Commands.parallel(
                Commands.run(() -> shooter.runShooter(shooterRps), shooter),
                kicker.kickerCommand())
                .withTimeout(timeoutSeconds)
                .andThen(shooter.stopCommand(), kicker.stopCommand());
    }

    /** Teleop shoot + whip assist while held. */
    public static Command shootWithWhip(
            ShooterSubsystem shooter, WhipSubsystem whip, double shooterRps) {
        return Commands.parallel(
                Commands.run(() -> shooter.runShooter(shooterRps), shooter),
                whip.whipCommand());
    }

    /** Runs shooter at a constant RPS while held. */
    public static Command runShooterAtRps(ShooterSubsystem shooter, double shooterRps) {
        return Commands.run(() -> shooter.runShooter(shooterRps), shooter);
    }
}
