package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WhipSubsystem;
import java.util.function.Supplier;

public final class ShooterCommands {

  private ShooterCommands() {}

  /**
   * Timed shoot sequence: run shooter and kicker in parallel, then stop both. * /** shoot + whip
   */
  public static Command shootWithWhipAndShake(
      ShooterSubsystem shooter,
      WhipSubsystem whip,
      KickerSubsystem kicker,
      IntakeDeploySubsystem intakeDeploy,
      double shooterRps) {

    return Commands.runOnce(intakeDeploy::shake, intakeDeploy)
        .andThen(
            Commands.parallel(
                Commands.run(() -> shooter.runShooter(shooterRps), shooter),
                whip.whipCommand(),

                // kicker now just runs simple logic based on shooter readiness inside command
                Commands.run(
                    () -> {
                      if (shooter.atTarget()) {
                        kicker.setKicker(0.5);
                      } else {
                        kicker.setKicker(0.0);
                      }
                    },
                    kicker)))
        .finallyDo(
            interrupted -> {
              shooter.stopShooter();
              kicker.stop();
              intakeDeploy.deploy();
            });
  }

  public static Command smartShoot(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      WhipSubsystem whip,
      IntakeDeploySubsystem intake,
      Supplier<Double> distanceMeters) {

    return Commands.runOnce(intake::shake, intake)
        .andThen(
            Commands.parallel(

                // WHIP ALWAYS RUNS
                whip.whipCommand(),

                // FULL CONTROL LOOP
                Commands.run(
                    () -> {
                      double distance = distanceMeters.get();

                      var profile = Constants.getShotProfile(distance);

                      // SHOOTER follows field model
                      shooter.runShooter(profile.shooterRps);

                      // KICKER reacts to shooter velocity vs profile threshold
                      double currentRps = shooter.getVelocityRps();

                      if (currentRps >= profile.kickerRpsThreshold) {
                        kicker.setKicker(0.5);
                      } else {
                        kicker.setKicker(0.0);
                      }
                    },
                    shooter,
                    kicker)))
        .finallyDo(
            interrupted -> {
              shooter.stopShooter();
              kicker.stop();
              intake.deploy();
            });
  }
}
