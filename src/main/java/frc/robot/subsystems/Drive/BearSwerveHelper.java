package frc.robot.subsystems.Drive;

import java.util.ArrayList;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.swervelib.Gyroscope;
import frc.swervelib.GyroscopeHelper;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.SwerveConstants;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveModule;
import frc.swervelib.SwerveSubsystem;
import frc.wpiClasses.QuadSwerveSim;

public class BearSwerveHelper {
        //Swerve Subsystem for auto and pathplanner
    public static SwerveDrivetrainModel createBearSwerve() {
        passConstants();
 
        ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);

        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L2,
                // This is the ID of the drive motor
                CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET,
                "FL"
        );
    
        // We will do the same for the other modules
        SwerveModule m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET,
                "FR"
        );
    
        SwerveModule m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                CanConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET,
                "BL"
        );
    
        SwerveModule m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET,
                "BR"
        );
    
        Gyroscope gyro = GyroscopeHelper.createPigeon2CAN(CanConstants.DRIVETRAIN_PIGEON_ID);
        modules.add(m_backRightModule);
        modules.add(m_frontLeftModule);
        modules.add(m_frontRightModule);
        modules.add(m_backLeftModule);

        return new SwerveDrivetrainModel(modules, gyro);
    }
    
        
    public static SwerveSubsystem createSwerveSubsystem(SwerveDrivetrainModel dt) {
        return new SwerveSubsystem(dt);        
    }

    private static void passConstants() {
        SwerveConstants.MAX_FWD_REV_SPEED_MPS = DriveConstants.MAX_FWD_REV_SPEED_MPS;
        SwerveConstants.MAX_STRAFE_SPEED_MPS = DriveConstants.MAX_STRAFE_SPEED_MPS;
        SwerveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC = DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC; 
        SwerveConstants.MAX_VOLTAGE = DriveConstants.MAX_VOLTAGE;
        SwerveConstants.DFLT_START_POSE = RobotConstants.DFLT_START_POSE;

        SwerveConstants.THETACONTROLLERkP = AutoConstants.THETACONTROLLERkP;
        SwerveConstants.TRAJECTORYXkP = AutoConstants.TRAJECTORYXkP;
        SwerveConstants.TRAJECTORYYkP = AutoConstants.TRAJECTORYYkP;
        SwerveConstants.THETACONTROLLERCONSTRAINTS = AutoConstants.THETACONTROLLERCONSTRAINTS;

        SwerveConstants.TRACKWIDTH_METERS = RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS;
        SwerveConstants.TRACKLENGTH_METERS = RobotConstants.DRIVETRAIN_WHEELBASE_METERS;
        SwerveConstants.MASS_kg = RobotConstants.MASS_kg;
        SwerveConstants.MOI_KGM2 = RobotConstants.MOI_KGM2;
        SwerveConstants.KINEMATICS = DriveConstants.DRIVETRAIN_KINEMATICS;
    }
}
