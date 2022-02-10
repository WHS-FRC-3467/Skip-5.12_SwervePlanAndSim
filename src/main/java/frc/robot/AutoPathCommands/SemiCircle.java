package frc.robot.AutoPathCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class SemiCircle extends SequentialCommandGroup {
  
  PathPlannerTrajectory forward = PathPlanner.loadPath("SemiCircle", 2.0, 3.0);
          
  public SemiCircle(SwerveSubsystem m_drive) {
    addCommands(
      m_drive.dt.createCommandForTrajectory(forward, m_drive)
    );
  }
}
