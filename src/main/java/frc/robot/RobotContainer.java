// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// This is being used by Team 6865, Manitoulin Metal

// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Telemetry.*;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.SimpleDriveAndSpinAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.subsystems.vision.VisionIOLimelight.*;
import frc.robot.subsystems.vision.VisionIOPhotonVision.*;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim.*;
import frc.robot.subsystems.vision.VisionMeasurement.*;
import frc.robot.subsystems.vision.VisionTemplate.*;
import frc.robot.subsystems.vision.VisionUtil.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

  // Subsystems
  private final Drive drive;

  PIDController XAlignController = new PIDController(Constants.X_ALIGN_P, 0, 0);
  PIDController YAlignController = new PIDController(Constants.Y_ALIGN_P, 0, 0);
  PIDController rotController = new PIDController(Constants.ROT_ALIGN_P, 0, 0);
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Field display
  private final Field2d field = new Field2d();

  // 2026 official AprilTag layout
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private boolean isRed;

  Timer dontSeeTagTimer = new Timer();
  private Object dontSeeTagTimerObject;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve template)
        // can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {
                  @Override
                  public void updateInputs(GyroIOInputs inputs) {
                    // Do nothing
                  }
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {
                    // Do nothing
                  }
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {
                    // Do nothing
                  }
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {
                    // Do nothing
                  }
                },
                new ModuleIO() {
                  @Override
                  public void updateInputs(ModuleIOInputs inputs) {
                    // Do nothing
                  }
                });
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // practice auto that drives forward and spins in place, for testing purposes
    autoChooser.addOption("Simple Drive Forward + Spin", new SimpleDriveAndSpinAuto(drive));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a GenericHID or one of its subclasses (for example,
   * edu.wpi.first.wpilibj.Joystick or XboxController), and then passing it to a button wrapper.
   */
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(Double.parseDouble(Constants.DONT_SEE_TAG_TIMEOUT_SECS));
  }

  public void AlignToTowerTagRelative(boolean isRed, Drive swerveSubsystem) {
    // Create or replace PID controllers normally (PIDController does not implement
    // AutoCloseable)
    XAlignController = new PIDController(Constants.X_ALIGN_P, 0, 0);
    YAlignController = new PIDController(Constants.Y_ALIGN_P, 0, 0);
    rotController = new PIDController(Constants.ROT_ALIGN_P, 0, 0);
    this.isRed = isRed;
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .leftTrigger(0.5) // activates when trigger pulled > 50%
        .onTrue(driveToTag26());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command driveToTag26() {

    // Get tag 26 pose from field layout
    var tagOptional = fieldLayout.getTagPose(26);

    if (tagOptional.isEmpty()) {
      return new InstantCommand(); // Do nothing if tag missing
    }

    Pose2d tagPose = tagOptional.get().toPose2d();

    // 2 feet in meters
    double offsetMeters = 0.9;

    // Create transform straight out from tag
    Transform2d offset = new Transform2d(new Translation2d(offsetMeters, 0), Rotation2d.kZero);

    Pose2d targetPose = tagPose.transformBy(offset);

    // Controllers
    PIDController xController = new PIDController(2.0, 0, 0);
    PIDController yController = new PIDController(2.0, 0, 0);

    ProfiledPIDController thetaController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(3, 3));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HolonomicDriveController controller =
        new HolonomicDriveController(xController, yController, thetaController);

    return new RunCommand(
            () -> {
              Pose2d currentPose = drive.getPose();

              var speeds =
                  controller.calculate(currentPose, targetPose, 0.0, targetPose.getRotation());

              drive.runVelocity(speeds);
            },
            drive)
        .until(
            () -> drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05)
        .andThen(() -> drive.stop());
  }
}
