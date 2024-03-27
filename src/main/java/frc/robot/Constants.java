// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.lang.Math;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
   
    // Field alignment angles
    public static final double SPEAKER_ANGLE = 180.0;
    public static final double RED_RAMP_SOURCE_ANGLE = 300.0;
    public static final double RED_ARM_SOURCE_ANGLE = 300.0 - 180;
    public static final double BLUE_AMP_ANGLE = 90.0;
    public static final double BLUE_RAMP_SOURCE_ANGLE = 60.0;
    public static final double BLUE_ARM_SOURCE_ANGLE = 60 + 180;
    public static final double RED_AMP_ANGLE = 270.0;

    public static final double FORWARD_ULTRASONIC_SENSOR_OFFSET = (10 * 25.4);  // 10 inches in mm
    public static final double REVERSE_ULTRASONIC_SENSOR_OFFSET = (12.25 * 25.4); // 6.5 inches in mm
    
    // Snow blower constants
    public static final double SNOW_BLOWER_START_ANGLE = 274.0;
    public static final double SNOW_BLOWER_SOURCE_ANGLE = 213.0;
    public static final double SNOW_BLOWER_AMP_ANGLE = 160.0;
    
    public static final int MAX_TALON_CMD_RATE_MSEC = 10;
    public static final double MARGIN_OF_SAFETY = 1.1;
<<<<<<< HEAD
    public static final double MAX_VELOCITY = 0.5; // for the chassis, meters per second
    public static final double SLOW_VELOCITY = 0.25; // for automatically slowing down when approaching a target
=======
    public static final double MAX_VELOCITY = 3.0; // for the chassis, meters per second
    public static final double SLOW_VELOCITY = 0.75; // for automatically slowing down when approaching a target
>>>>>>> 71a04ab9e611131c0e393baa2b989be44e41245f
    public static final double MAX_WHEEL_VELOCITY = 3.0; // for any wheel, meters per second

    // My Murphy's Robot
    public static final boolean payloadsEnabled = false;
    public static final double WHEEL_OFFSET_X = 12.0 * 0.0254; // converted to meters
    public static final double WHEEL_OFFSET_Y = 12.0 * 0.0254; // converted to meters

    // TEAM 6072's Robot
    //public static final boolean payloadsEnabled = true;
    //public static final double WHEEL_OFFSET_X = 12.75 * 0.0254; // converted to meters
    //public static final double WHEEL_OFFSET_Y = 12.75 * 0.0254; // converted to meters

    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;
    public static final double kWheelDiameterMeters = 0.0762;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final int drivingMotorCurrentLimit = 40;
    public static final int steeringMotorCurrentLimit = 20;

    // Chassis configuration
    public static final double kTrackWidth = 2 * WHEEL_OFFSET_X;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 2 * WHEEL_OFFSET_Y;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class AutoConstants {
        public static final double kSimplePullForwardSpeed = 1.0;
        public static final double kSimplePullForwardAccel = 1.0;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    }

    public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0),
        0.25,
        Math.sqrt(WHEEL_OFFSET_X * WHEEL_OFFSET_X + WHEEL_OFFSET_Y * WHEEL_OFFSET_Y),
        new ReplanningConfig()
    );
}

