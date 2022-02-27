// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveIntake extends CommandBase {
  /** Creates a new AutoDriveIntake. */
  IntakeSubsystem m_intake;
  double m_speed;
  public AutoDriveIntake(IntakeSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(m_intake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.driveIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.driveIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
