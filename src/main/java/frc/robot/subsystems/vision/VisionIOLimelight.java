package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** Real Limelight implementation. */
public class VisionIOLimelight implements VisionIO {

  private final String cameraName;

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber tvSubscriber;

  // private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private int flushCounter = 0;

  public VisionIOLimelight(String cameraName, Supplier<Rotation2d> rotationSupplier) {

    this.cameraName = cameraName;
    this.rotationSupplier = rotationSupplier;

    var table = NetworkTableInstance.getDefault().getTable(cameraName);

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);

    // megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});

    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000.0) < 250.0;

    boolean hasTarget = tvSubscriber.get() == 1.0;
    inputs.hasTargets = hasTarget;

    inputs.latestTargetObservation =
        hasTarget
            ? new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()),
                Rotation2d.fromDegrees(tySubscriber.get()))
            : new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    inputs.targetSpacePose = LimelightHelpers.getBotPose3d_TargetSpace(cameraName);
    orientationPublisher.accept(new double[] {rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0});

    if (++flushCounter >= 5) {
      NetworkTableInstance.getDefault().flush();
      flushCounter = 0;
    }

    List<PoseObservation> observations = new ArrayList<>();
    Set<Integer> tagIds = new HashSet<>();

    inputs.targetSpacePose = LimelightHelpers.getBotPose3d_TargetSpace(cameraName);

    // readQueue(megatag1Subscriber, observations, tagIds,
    // PoseObservationType.MEGATAG_1);
    readQueue(megatag2Subscriber, observations, tagIds, PoseObservationType.MEGATAG_2);

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private void readQueue(
      DoubleArraySubscriber sub,
      List<PoseObservation> observations,
      Set<Integer> tagIds,
      PoseObservationType type) {

    for (var sample : sub.readQueue()) {

      if (sample.value.length == 0) continue;
      if (sample.value.length < 11) continue;

      int tagCount = (int) sample.value[7];
      if (tagCount <= 0) continue;

      double avgTagDistance = sample.value[9];
      if (avgTagDistance > 6.0) continue;

      for (int i = 11; i < sample.value.length; i += 7) {
        tagIds.add((int) sample.value[i]);

        // for temp debugging - record the pose of each individual tag observation, even if we
        // reject it for pose estimation
        org.littletonrobotics.junction.Logger.recordOutput(
            "Vision/DebugPose", parsePose(sample.value).toPose2d());
      }

      observations.add(
          new PoseObservation(
              sample.timestamp * 1.0e-6 - sample.value[6] * 1.0e-3,
              parsePose(sample.value),
              type == PoseObservationType.MEGATAG_1
                  ? (sample.value.length >= 18 ? sample.value[17] : 0.0)
                  : 0.0,
              (int) sample.value[7],
              sample.value[9],
              type));
    }
  }

  private static Pose3d parsePose(double[] raw) {
    return new Pose3d(
        raw[0],
        raw[1],
        raw[2],
        new Rotation3d(
            Units.degreesToRadians(raw[3]),
            Units.degreesToRadians(raw[4]),
            Units.degreesToRadians(raw[5])));
  }
}
