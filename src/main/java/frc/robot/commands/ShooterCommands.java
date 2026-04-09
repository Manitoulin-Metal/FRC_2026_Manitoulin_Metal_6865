package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WhipSubsystem;

public final class ShooterCommands {

    private ShooterCommands() {
    }

    /** Timed shoot sequence: run shooter and kicker in parallel, then stop both. */
    public static Command timedShoot(
            ShooterSubsystem shooter,
            KickerSubsystem kicker,
            IntakeDeploySubsystem intakeDeploy,
            double shooterRps,
            double timeoutSeconds) {
        return Commands.runOnce(intakeDeploy::shake, intakeDeploy)
                .andThen(
                        Commands.parallel(
                                Commands.run(() -> shooter.runShooter(shooterRps), shooter),
                                kicker.kickerCommand())
                                .withTimeout(timeoutSeconds))
                .finallyDo(interrupted -> {
                    shooter.stopShooter();
                    intakeDeploy.stow();
                });
    }

    /** Teleop shoot + whip assist while held. */
    public static Command shootWithWhip(
            ShooterSubsystem shooter,
            WhipSubsystem whip,
            IntakeDeploySubsystem intakeDeploy,
            double shooterRps) {
        return Commands.runOnce(intakeDeploy::shake, intakeDeploy)
                .andThen(
                        Commands.parallel(
                                Commands.run(() -> shooter.runShooter(shooterRps), shooter),
                                whip.whipCommand()))
                .finallyDo(interrupted -> {
                    shooter.stopShooter();
                    intakeDeploy.stow();
                });
    }

    /** Runs shooter at a constant RPS while held. */
    public static Command runShooterAtRps(
            ShooterSubsystem shooter,
            IntakeDeploySubsystem intakeDeploy,
            double shooterRps) {
        return Commands.runOnce(intakeDeploy::shake, intakeDeploy)
                .andThen(Commands.run(() -> shooter.runShooter(shooterRps), shooter))
                .finallyDo(interrupted -> {
                    shooter.stopShooter();
                    intakeDeploy.stow();
                });
    }
}
