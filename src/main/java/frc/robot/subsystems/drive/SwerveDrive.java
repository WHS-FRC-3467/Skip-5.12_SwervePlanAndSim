package frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.swervelib.SwerveInput;
import frc.swervelib.SwerveSubsystem;

public class SwerveDrive extends CommandBase {
    
    SwerveSubsystem m_swerveSubsystem;
    DoubleSupplier m_translationXSupplier;
    DoubleSupplier m_translationYSupplier;
    DoubleSupplier m_rotationSupplier;

    //Constructor for SwerveDrive
    public SwerveDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {

        m_swerveSubsystem = swerveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;

        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void execute() {

      SwerveInput swerveInput = new SwerveInput(
        modifyAxis(m_translationXSupplier.getAsDouble()),
        modifyAxis(m_translationYSupplier.getAsDouble()),
        modifyAxis(m_rotationSupplier.getAsDouble())
      ); 

      m_swerveSubsystem.dt.setModuleStates(swerveInput);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.dt.setModuleStates(new SwerveInput(0.0, 0.0, 0.0));    

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