package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase{
      //Initialize Variables
    DriveSubsystem m_driveSubsystem;
    DoubleSupplier m_translationXSupplier;
    DoubleSupplier m_translationYSupplier;
    DoubleSupplier m_rotationSupplier;

    SlewRateLimiter m_XRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter m_YRateLimiter = new SlewRateLimiter(0.5);

    //Constructor for SwerveDrive
    public SwerveDrive(DriveSubsystem driveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        m_driveSubsystem = driveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement 
        m_driveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                m_XRateLimiter.calculate(modifyAxis(m_translationXSupplier.getAsDouble())),
                m_YRateLimiter.calculate(modifyAxis(m_translationYSupplier.getAsDouble())),
                modifyAxis(m_rotationSupplier.getAsDouble()),
                m_driveSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private static double modifyAxis(double value) {
      
        // Apply Deadband
          double dband = DriveConstants.STICK_DEADBAND;
          if (Math.abs(value) > dband) {
            if (value > 0.0) {
                return (value - dband) / (1.0 - dband);
            } else {
                return (value + dband) / (1.0 - dband);
            }
          } else {
            value = 0.0;
          }
        
        // Square the axis for better control range at low speeds
        value = Math.copySign(value * value, value);
    
        return value;
      }
    

}
