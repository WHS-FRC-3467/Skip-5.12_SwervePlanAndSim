package frc.robot.autocommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Swerve1 extends SequentialCommandGroup {
  PathPlannerTrajectory ppTraj = PathPlanner.loadPath("Swerve2", 2.0, 3.0);
  public Swerve1(DriveSubsystem m_drive) {
    addCommands(
      m_drive.dt.createCommandForTrajectory(ppTraj, m_drive)
    );
  }
}
