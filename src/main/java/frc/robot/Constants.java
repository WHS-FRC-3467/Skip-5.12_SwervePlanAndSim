// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CanConstants{
        //drivebase can ids 1-12
        //non drivebase can constants

    }
	
    public static final class RobotConstants {
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ROBOT PHYSICAL CONSTANTS
        // Robot physical dimensions and mass quantities.
        public static final double MASS_kg = Units.lbsToKilograms(140);
        public static final double MOI_KGM2 = 1.0/12.0 * RobotConstants.MASS_kg * Math.pow((DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

        public static final double QUIESCENT_CURRENT_DRAW_A = 2.0; //Misc electronics
        public static final double BATTERY_NOMINAL_VOLTAGE = 13.2; //Nicely charged battery
        public static final double BATTERY_NOMINAL_RESISTANCE = 0.040; //40mOhm - average battery + cabling

        // Assumed starting location of the robot. Auto routines will pick their own location and update this.
        static public final Pose2d DFLT_START_POSE = new Pose2d(Units.feetToMeters(24.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(0));

        // Location of vision camera relative to robot center - currently front middle.
        static public final Transform2d robotToCameraTrans = new Transform2d(new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS/2, 0), new Rotation2d(0.0));

        // Vision Camera
        static public final String PHOTON_CAM_NAME = "MainCamera";
    }
    public static final class DriveConstants{
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5334; // DONEFIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334; // DONEFIXME Measure and set wheelbase

        public static final SwerveDriveKinematics DRIVETRAIN_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        public static final double WHEEL_DIAMETER_METERS = 0.10033; // .10033 = ~4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final int DRIVETRAIN_PIGEON_ID = 6; // DONEFIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // DONEFIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // DONEFIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; // DONEFIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(71.719); // DONEFIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // DONEFIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // DONEFIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; // DONEFIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(295.751); // DONEFIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // DONEFIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4; // DONEFIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // DONEFIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(329.301); // DONEFIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // DONEFIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // DONEFIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; // DONEFIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(288.295); // DONEFIXME Measure and set back right steer offset        

        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(360.0);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
        static public final int ENC_PULSE_PER_REV = 2048; // TalonFX integrated sensor
        static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
        static public final int STEER_ENC_COUNTS_PER_MODULE_REV = 4096; // CANCoder
        static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        static public final double steer_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(STEER_ENC_COUNTS_PER_MODULE_REV));
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
