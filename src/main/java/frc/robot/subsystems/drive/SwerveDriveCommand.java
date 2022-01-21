package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.control.XboxControllerEE;

public class SwerveDriveCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final XboxControllerEE m_controller;
    private static final SendableChooser<String> driverChooser = new SendableChooser<>();
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    private double m_translationX;
    private double m_translationY;
    private double m_rotation;

    public SwerveDriveCommand(DriveSubsystem driveSubsystem,
                               XboxControllerEE controller) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_controller = controller;

        // Control Scheme Chooser
        driverChooser.setDefaultOption("Both Sticks", "Both Sticks");
        driverChooser.addOption("Left Stick and Triggers", "Left Stick and Triggers");
        driverChooser.addOption("Split Sticks and Triggers", "Split Sticks and Triggers");
        driverChooser.addOption("Gas Pedal", "Gas Pedal");
        SmartDashboard.putData("Driver Chooser", driverChooser);

        // Control Orientation Chooser
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        switch (driverChooser.getSelected()) {
            case "Both Sticks":
              m_translationX = modifyAxis(-m_controller.getLeftYwDB());
              m_translationY = modifyAxis(-m_controller.getLeftXwDB());
              m_rotation = modifyAxis(-m_controller.getRightXwDB());
              break;
            case "Left Stick and Triggers":
              m_translationX = modifyAxis(-m_controller.getLeftYwDB());
              m_translationY = modifyAxis(-m_controller.getLeftXwDB());
              m_rotation = m_controller.TriggerCombined();
              break;
            case "Split Sticks and Triggers":
              m_translationX = modifyAxis(-m_controller.getLeftYwDB());
              m_translationY = modifyAxis(-m_controller.getRightXwDB());
              m_rotation = m_controller.TriggerCombined();
              break;
            case "Gas Pedal":
              m_translationX = modifyAxis(-m_controller.getLeftYwDB());
              m_translationY = modifyAxis(-m_controller.getLeftXwDB());
              double angle = calculateTranslationDirection(m_translationX, m_translationY);
              m_translationX = Math.cos(angle) * m_controller.getRightTriggerAxis();
              m_translationY = Math.sin(angle) * m_controller.getRightTriggerAxis();
              m_rotation = modifyAxis(m_controller.getRightXwDB());
              break;
        }

        switch (orientationChooser.getSelected()) {
          case "Field Oriented":
            m_driveSubsystem.dt.setModuleStates(
                DriveConstants.DRIVETRAIN_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationX * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                            m_translationY * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                            m_rotation * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                            m_driveSubsystem.dt.getGyroscopeRotation()
                    )
                )    
            );
            break;
          case "Robot Oriented":
            m_driveSubsystem.dt.setModuleStates(
              DriveConstants.DRIVETRAIN_KINEMATICS.toSwerveModuleStates(
                  new ChassisSpeeds(
                          m_translationX * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                          m_translationY * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                          m_rotation * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                  )
              )    
            );
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.dt.setModuleStates(
          DriveConstants.DRIVETRAIN_KINEMATICS.toSwerveModuleStates(  
            new ChassisSpeeds(0.0, 0.0, 0.0)
          )    
        );
    }

  private static double modifyAxis(double value) {
      // Square the axis
      value = Math.copySign(value * value, value);

      return value;
  }

  /**
     * Calculates the angle of translation set by the left stick.
     *
     * @return The angle of translation. 0 corresponds to forwards, and positive
     *     corresponds to counterclockwise.
    */
  private double calculateTranslationDirection(double x, double y) {
    // Calculate the angle.
    // Swapping x/y and inverting y because our coordinate system has +x forwards and -y right
    return Math.atan2(x, -y);
  }
}