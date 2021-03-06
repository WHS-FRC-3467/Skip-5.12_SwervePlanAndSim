package frc.robot.AutoPathCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class Center5Ball extends SequentialCommandGroup {
    public Center5Ball(SwerveSubsystem m_swerve) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("5ball", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownPose(trajectory1.getInitialPose())),

            //turn shooter on to get up to speed
            /* new InstantCommand(() -> {m_shooter.setSetpoint(ShooterConstants.kShooter4);
                m_shooter.enable();}, m_shooter), */

            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve)
        );
    }
}
