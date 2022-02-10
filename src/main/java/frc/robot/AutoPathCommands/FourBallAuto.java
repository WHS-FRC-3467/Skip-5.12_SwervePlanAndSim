package frc.robot.AutoPathCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
  PathPlannerTrajectory ppTraj = PathPlanner.loadPath("4BallAutoRevB", 2.0, 3.0);
  public FourBallAuto(SwerveSubsystem m_drive) {
    addCommands(
      new InstantCommand(() -> m_drive.dt.setKnownPose(ppTraj.getInitialPose())),
      m_drive.dt.createCommandForTrajectory(ppTraj, m_drive)
    );
  }
}
