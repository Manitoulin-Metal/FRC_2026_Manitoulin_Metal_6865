// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// This is being used by Team 6865, Manitoulin Metal

// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {

  private final String name;

  private final Supplier<Rotation2d> rotationSupplier;

  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;

  private final DoubleSubscriber txSubscriber;

  private final DoubleSubscriber tySubscriber;

  private final DoubleArraySubscriber megatag1Subscriber;

  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;

    var table = NetworkTableInstance.getDefault().getTable(name);

    this.rotationSupplier = rotationSupplier;

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);

    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);

    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});

    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Check connection based on latency (still useful)
    inputs.connected =
        ((RobotController.getFPGATime() - txSubscriber.getLastChange()) / 1000) < 250;

    // ------------------ Basic target info ------------------
    double tx = txSubscriber.get();
    double ty = tySubscriber.get();

    // Only mark a tag if Limelight sees one (tv==1)
    boolean hasTarget = tx != 0.0 || ty != 0.0;
    inputs.latestTargetObservation =
        new TargetObservation(Rotation2d.fromDegrees(tx), Rotation2d.fromDegrees(ty));

    // ------------------ Pose observations from botpose_wpiblue ------------------
    var rawSamples =
        NetworkTableInstance.getDefault()
            .getTable(name)
            .getDoubleArrayTopic("botpose_wpiblue")
            .subscribe(new double[] {})
            .readQueue();

    List<PoseObservation> poseObservations = new LinkedList<>();
    Set<Integer> tagIds = new HashSet<>();

    for (var raw : rawSamples) {
      if (raw.value.length < 8) continue; // ensure valid data

      Pose3d pose =
          new Pose3d(
              raw.value[0],
              raw.value[1],
              raw.value[2],
              new Rotation3d(
                  Units.degreesToRadians(raw.value[3]),
                  Units.degreesToRadians(raw.value[4]),
                  Units.degreesToRadians(raw.value[5])));

      int tagCount = (int) raw.value[7]; // number of tags contributing
      double avgDistance = (tagCount > 0 && raw.value.length >= 9) ? raw.value[8] : 0.0;

      // Collect tag IDs (starts at index 11 for each tag, step 7)
      for (int i = 11; i < raw.value.length; i += 7) {
        tagIds.add((int) raw.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              raw.timestamp * 1.0e-6 - 0.0, // timestamp, you could use latency if needed
              pose,
              0.0, // ambiguity, can ignore for multi-tag
              tagCount,
              avgDistance,
              PoseObservationType.MEGATAG_2));
    }

    // Save observations
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // Save tag IDs
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {

    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
