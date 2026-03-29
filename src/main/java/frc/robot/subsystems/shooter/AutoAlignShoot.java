package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;
