package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

@SuppressWarnings("unused")
public class Shoot3BallsCommand extends SequentialCommandGroup {
  public Shoot3BallsCommand(ShooterSubsystem shooter, KickerSubsystem kicker) {
    addRequirements(shooter, kicker);

    addCommands(
      // Shot 1
      Commands.parallel(
        Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
        kicker.kickerCommand(0.3)
      ).withTimeout(2.0),
      // Shot 2
      Commands.parallel(
        Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
        kicker.kickerCommand(0.3)
      ).withTimeout(2.0),
      // Shot 3
      Commands.parallel(
        Commands.run(() -> shooter.runShooter(Constants.AUTO_SHOOT_RPS), shooter),
        kicker.kickerCommand(0.3)
      ).withTimeout(2.0),
      // Stop
      shooter.stopCommand().andThen(kicker.stopCommand())
    );
  }
}

