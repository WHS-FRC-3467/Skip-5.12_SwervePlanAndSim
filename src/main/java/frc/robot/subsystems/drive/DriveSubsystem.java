package frc.robot.subsystems.Drive;
// Copyright (c) FIRST and other WPILib contributors.



// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.swervelib.SwerveSubsystem;

import static frc.robot.Constants.CanConstants;
import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

        // Provide access to the Drive motor controllers
        TalonFX m_frontLeftDriveMotor = new TalonFX(CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
        TalonFX m_frontRightDriveMotor = new TalonFX(CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        TalonFX m_backLeftDriveMotor = new TalonFX(CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR);
        TalonFX m_backRightDriveMotor = new TalonFX(CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR);

        // Provide access to the Steering motor encoders
        CANCoder m_frontLeftCanCoder = new CANCoder(CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER);
        CANCoder m_backLeftCanCoder = new CANCoder(CanConstants.BACK_LEFT_MODULE_STEER_ENCODER);
        CANCoder m_frontRightCanCoder = new CANCoder(CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER);
        CANCoder m_backRightCanCoder = new CANCoder(CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER);

        private SwerveSubsystem m_swerveSubsystem;

        public DriveSubsystem(SwerveSubsystem swerveSubsystem) {

                m_swerveSubsystem = swerveSubsystem;
                
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // Setup Steering motor encoders to always boot to absolute position
                m_frontLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_backLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_frontRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_backRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

                // Setup Drive motor encoders to always boot to zero
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

                // Configure Drive motors to use Integrated (Internal) encoders
                m_frontLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_backLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_frontRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_backRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

                m_frontLeftDriveMotor.setInverted(false);
                m_backLeftDriveMotor.setInverted(false);
                m_frontRightDriveMotor.setInverted(false);
                m_backRightDriveMotor.setInverted(false);
        }

        public double getAverageEncoder(){
                //returns in 2048/rotation
                return (m_frontLeftDriveMotor.getSelectedSensorPosition() +
                        m_frontRightDriveMotor.getSelectedSensorPosition() +
                        m_backLeftDriveMotor.getSelectedSensorPosition() +
                        m_backRightDriveMotor.getSelectedSensorPosition())/4;
        }

        public double meterToEncoderTicks(double meters){
                ModuleConfiguration modConfig = m_swerveSubsystem.dt.getRealModules().get(0).getModuleConfiguration();
                return meters * (2048/(modConfig.getDriveReduction() * modConfig.getWheelDiameter() * Math.PI));
        }

        public double encoderTicksToMeter(double ticks){
                ModuleConfiguration modConfig = m_swerveSubsystem.dt.getRealModules().get(0).getModuleConfiguration();
                return ticks * ((modConfig.getDriveReduction() * modConfig.getWheelDiameter() * Math.PI)/2048);
        }

        public void setState(double speed, double angle){
                // Module state (all four will be identical)
                var moduleState = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));

                // Convert to chassis speeds
                var chassisSpeeds = DriveConstants.DRIVETRAIN_KINEMATICS.toChassisSpeeds(moduleState, moduleState, moduleState, moduleState);

                // Update the SwerveSubsytem's SwerveModuleStates[] matrix
                // These will be picked up by the SwerveSubsystem's periodic() method()
                m_swerveSubsystem.dt.setModuleStates(DriveConstants.DRIVETRAIN_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
        }
}