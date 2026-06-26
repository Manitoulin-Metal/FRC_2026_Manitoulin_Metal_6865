package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {

  private final IntakeDeploySubsystem intake;
  private final ClimbSubsystem climb;

  public RobotVisualizer(IntakeDeploySubsystem intake, ClimbSubsystem climb) {

    this.intake = intake;
    this.climb = climb;
  }

  public void update() {
    double intakeAngleDeg = intake.getAngleDegrees();

    double climberPos = climb.getPosition();

    Logger.recordOutput("Visualizer/IntakeAngle", intakeAngleDeg);
    Logger.recordOutput("Visualizer/ClimberPos", climberPos);

    // Normalize climber travel from 0.0 to 1.0
    double climberPercent = climberPos / Constants.Climb.Motion.UP_TARGET_ROTATIONS;

    Logger.recordOutput(
        "ComponentPoses",
        new Pose3d[] {

          // Component 0 = Intake
          new Pose3d(0.3, 0.15, 0.225, new Rotation3d(0, Math.toRadians(intakeAngleDeg), 0)),

          // Component 1 = Climber
          new Pose3d(0.0, 0.0, -0.15 * climberPercent, new Rotation3d())
        });
  }
}
