package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  // ===================== CONTRACT =====================
  public static final double ODOMETRY_FREQUENCY =
      TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;

  public static final Lock odometryLock = new ReentrantLock();

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // ===================== CONFIG =====================
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  // ===================== IO =====================
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  private final Alert gyroAlert =
      new Alert("Gyro disconnected — using kinematic fallback", AlertType.kError);

  @SuppressWarnings("unused")
  private Vision vision;

  // ===================== ESTIMATOR =====================
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());

  private Rotation2d rawGyro = Rotation2d.kZero;

  private final SwerveModulePosition[] lastPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator estimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyro, lastPositions, Pose2d.kZero);

  private SysIdRoutine sysId;

  public SysIdRoutine getSysId() {
    return sysId;
  }

  // ===================== CONSTRUCTOR =====================
  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {

    this.gyroIO = gyroIO;

    modules[0] = new Module(fl, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(fr, 1, TunerConstants.FrontRight);
    modules[2] = new Module(bl, 2, TunerConstants.BackLeft);
    modules[3] = new Module(br, 3, TunerConstants.BackRight);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    PhoenixOdometryThread.getInstance().start();

    configurePathPlanner();
    configureSysId();

    // ===================== SIM START POSE =====================
    if (Constants.currentMode == Constants.Mode.SIM) {
      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

      Pose2d startPose =
          (alliance == Alliance.Red)
              ? new Pose2d(12.0, 6.0, Rotation2d.fromDegrees(180))
              : new Pose2d(3.0, 6.0, Rotation2d.fromDegrees(0));

      setPose(startPose);
    }
  }

  // ===================== PATHPLANNER =====================
  private void configurePathPlanner() {
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        p -> Logger.recordOutput("Odometry/Trajectory", p.toArray(Pose2d[]::new)));

    PathPlannerLogging.setLogTargetPoseCallback(
        p -> Logger.recordOutput("Odometry/TrajectorySetpoint", p));
  }

  private void configureSysId() {
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, s -> Logger.recordOutput("Drive/SysIdState", s.toString())),
            new SysIdRoutine.Mechanism(v -> runCharacterization(v.in(Volts)), null, this));
  }

  // ===================== PERIODIC =====================
  @Override
  public void periodic() {

    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);

      for (var m : modules) m.periodic();

    } finally {
      odometryLock.unlock();
    }

    updateOdometry();
  }

  // ===================== ODOMETRY =====================
  private void updateOdometry() {

    double[] timestamps = modules[0].getOdometryTimestamps();

    for (int i = 0; i < timestamps.length; i++) {

      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      SwerveModulePosition[] deltas = new SwerveModulePosition[4];

      for (int m = 0; m < 4; m++) {
        positions[m] = modules[m].getOdometryPositions()[i];

        deltas[m] =
            new SwerveModulePosition(
                positions[m].distanceMeters - lastPositions[m].distanceMeters, positions[m].angle);

        lastPositions[m] = positions[m];
      }

      if (gyroInputs.connected && gyroInputs.odometryYawPositions.length > i) {
        rawGyro = gyroInputs.odometryYawPositions[i];
        gyroAlert.set(false);
      } else {
        rawGyro = rawGyro.plus(new Rotation2d(kinematics.toTwist2d(deltas).dtheta));
        gyroAlert.set(true);
      }

      estimator.updateWithTime(timestamps[i], rawGyro, positions);
    }

    // ===================== CORE VISUALIZATION LOGS =====================
    Pose2d pose = estimator.getEstimatedPosition();

    Logger.recordOutput("RobotPose", pose);
    Logger.recordOutput("Odometry/Robot", pose);
    Logger.recordOutput("Field/Robot", pose);

    Logger.recordOutput("Odometry/RobotRotationDeg", pose.getRotation().getDegrees());
  }

  // ===================== PUBLIC API =====================
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    estimator.resetPosition(rawGyro, getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    estimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  public void runVelocity(ChassisSpeeds speeds) {

    var states = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, TunerConstants.kSpeedAt12Volts);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }

    Logger.recordOutput("SwerveStates/Setpoints", states);
  }

  public void runCharacterization(double volts) {
    for (var m : modules) m.runCharacterization(volts);
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];

    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }

    kinematics.resetHeadings(headings);
    stop();
  }

  // ===================== HELPERS =====================
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] out = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) out[i] = modules[i].getPosition();
    return out;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] out = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) out[i] = modules[i].getState();
    return out;
  }

  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  // ===============================
  // COMPATIBILITY HELPERS
  // ===============================

  public ChassisSpeeds toFieldRelativeSpeeds(double x, double y, double omega) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, getRotation());
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }
}
