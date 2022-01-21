// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.swervelib.Gyroscope;
import frc.swervelib.GyroscopeHelper;
import frc.swervelib.Mk3SwerveModuleHelper;
import frc.swervelib.sim.SwerveDrivetrainModel;
import frc.wpiClasses.QuadSwerveSim;
import frc.swervelib.SdsModuleConfigurations;
import frc.swervelib.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.util.ArrayList;

public class DriveSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // DONEFIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
          SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private SwerveModuleState[] states;

  // Simulation declarations
  ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
  PDPSim pdp;
  public SwerveDrivetrainModel dt;

  public DriveSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // DONEFIXME Setup motor configuration
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk3SwerveModuleHelper.GearRatio.FAST,
            // This is the ID of the drive motor
            DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET, "FL"
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET, "FR"
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET, "BL"
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET, "BR"
    );

    Gyroscope gyro = GyroscopeHelper.createPigeonCAN(DriveConstants.DRIVETRAIN_PIGEON_ID);

    modules.add(m_frontLeftModule);
    modules.add(m_frontRightModule);
    modules.add(m_backLeftModule);
    modules.add(m_backRightModule);
    dt = new SwerveDrivetrainModel(modules, gyro);

    // Simulation Setup
    if (RobotBase.isSimulation()) {
      
    }
  }

  @Override
  public void periodic() {
    states = dt.getSwerveModuleStates();

    if (states != null) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

      m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());


    }
    dt.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    dt.update(DriverStation.isDisabled(), 13.2);
  }
}
