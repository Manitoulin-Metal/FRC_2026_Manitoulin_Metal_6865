package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public boolean hasTag(int id) {
    return inputs.latestTargetObservation != null;
  }

  public double getTX() {
    return inputs.latestTargetObservation.tx().getDegrees();
  }

  public double getTY() {
    return inputs.latestTargetObservation.ty().getDegrees();
  }
}
