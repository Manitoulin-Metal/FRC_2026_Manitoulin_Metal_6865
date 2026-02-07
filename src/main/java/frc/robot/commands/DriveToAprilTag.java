package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

public class DriveToAprilTag extends Command {
  private final Drive drive;
  private final VisionSubsystem vision;
  private final int targetID;

  private final PIDController turnPID = new PIDController(0.03, 0, 0);

  public DriveToAprilTag(Drive drive, VisionSubsystem vision, int targetID) {
    this.drive = drive;
    this.vision = vision;
    this.targetID = targetID;

    addRequirements(drive);

    turnPID.setTolerance(1.0);
  }

  @Override
  public void execute() {
    if (vision.hasTag(targetID)) {
      // Tag found
      double tx = vision.getTX();
      double turn = turnPID.calculate(tx, 0);

      double forward = 0.6; // approach speed

      drive.runVelocity(new ChassisSpeeds(forward, 0, turn));

    } else {
      // Search for tag
      drive.runVelocity(new ChassisSpeeds(0, 0, 0.4));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
