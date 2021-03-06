// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoPathCommands.TwoBallPath;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;
import frc.swervelib.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallTerminalAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallTerminal. */
  DriveSubsystem m_drive;
  SwerveSubsystem m_swerve;
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;

  public ThreeBallTerminalAuto(DriveSubsystem drive, SwerveSubsystem swerve, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake) {
    m_drive = drive;
    m_swerve = swerve;
    m_intake = intake;
    m_tower = tower;
    m_shooter = shooter;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(m_shooter, m_tower, ShooterConstants.upperHubVelocity),
      new InstantCommand(m_intake::intakeDeploy, m_intake),
      new ParallelCommandGroup(
                                new TwoBallPath(m_swerve),
                                new AutoDriveIntake(m_intake, 0.75)),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.upperHubVelocity)
    );

  }
}
