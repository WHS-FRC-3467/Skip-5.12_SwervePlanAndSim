package frc.robot.autocommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class MoveForward extends SequentialCommandGroup {
  PathPlannerTrajectory forward = PathPlanner.loadPath("MoveForward", 2.0, 3.0);
  public MoveForward(DriveSubsystem m_drive) {
    addCommands(
      m_drive.dt.createCommandForTrajectory(forward, m_drive)
    );
  }
}
