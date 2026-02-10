// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
// This is being used by Team 6865, Manitoulin Metal

// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.Telemetry.*;
import frc.robot.Robot;
import frc.robot.Main;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.intakedeploy.IntakeDeploySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXS;
import frc.robot.subsystems.intake.intakeroller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.VisionUtil.*;
import frc.robot.subsystems.vision.VisionTemplate.*;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.subsystems.vision.VisionMeasurement.*;
import frc.robot.subsystems.vision.VisionIOLimelight.*;
import frc.robot.subsystems.vision.VisionIOPhotonVision.*;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim.*;
import frc.robot.subsystems.vision.VisionConstants.*;

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
        // TalonFXS controller connected to a CANdi with a PWM encoder. The implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark swerve template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    // Create or replace PID controllers normally (PIDController does not implement AutoCloseable)
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
