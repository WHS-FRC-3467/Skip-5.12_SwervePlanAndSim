package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

import java.util.function.DoubleSupplier;
import frc.swervelib.SwerveInput;
import frc.swervelib.SwerveSubsystem;

public class SwerveDrive extends CommandBase{
      //Initialize Variables
    SwerveSubsystem m_swerveSubsystem;
    DoubleSupplier m_translationXSupplier;
    DoubleSupplier m_translationYSupplier;
    DoubleSupplier m_rotationSupplier;
    double m_deadBand = DriveConstants.STICK_DEADBAND;
    SlewRateLimiter m_XRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter m_YRateLimiter = new SlewRateLimiter(0.5);
    
    //Constructor for SwerveDrive
    public SwerveDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {

        m_swerveSubsystem = swerveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;

        addRequirements(m_swerveSubsystem);

        if (Robot.isSimulation()) {
            m_deadBand = 0.20;
        }
    }

    @Override
    public void execute() {

      SwerveInput swerveInput = new SwerveInput(
        m_XRateLimiter.calculate(modifyAxis(m_translationXSupplier.getAsDouble())),
        m_YRateLimiter.calculate(modifyAxis(m_translationYSupplier.getAsDouble())),
        modifyAxis(m_rotationSupplier.getAsDouble())
      ); 

      m_swerveSubsystem.dt.setModuleStates(swerveInput);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.dt.setModuleStates(new SwerveInput(0.0, 0.0, 0.0));    

    }

    private double modifyAxis(double value) {
      
        // Apply Deadband
          if (Math.abs(value) > m_deadBand) {
            if (value > 0.0) {
                return (value - m_deadBand) / (1.0 - m_deadBand);
            } else {
                return (value + m_deadBand) / (1.0 - m_deadBand);
            }
          } else {
            value = 0.0;
          }
        
        // Square the axis for better control range at low speeds
        value = Math.copySign(value * value, value);
    
        return value;
      }
    

}
