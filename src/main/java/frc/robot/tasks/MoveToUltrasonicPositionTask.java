package frc.robot.tasks;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.FrontBackUltrasonicSubsystem;
import frc.robot.subsystems.NavSubsystem;

public class MoveToUltrasonicPositionTask {
    FrontBackUltrasonicSubsystem m_fbsys;

    public MoveToUltrasonicPositionTask (FrontBackUltrasonicSubsystem fbSys) {
        m_fbsys = fbSys;
    }

    public ChassisSpeeds moveToForwardSensorState(double distance, double desiredAngle) {
        double currentAngle;
        double throttleFactor;
        double targetOffset;
        double range;
        double shortestPathOverride = 1.0;

        double maxAngularRate = (90.0 - 20.0) / 180.0 * Math.PI;  // 70 deg/sec in radians

        // Determine the angle offset and correct that first
        currentAngle = NavSubsystem.getContinuousAngle();

        double revs = Math.floor(currentAngle / 360.0);
        currentAngle = currentAngle - (revs * 360.0);

//        Logger.recordOutput("Desired Angle", desiredAngle);
//        Logger.recordOutput("Current Angle", currentAngle);

        targetOffset = (desiredAngle - currentAngle);
//        Logger.recordOutput("Target Offset", targetOffset);

        if(targetOffset > 180.0) {
            shortestPathOverride   = -1.0;
            targetOffset -= 180.0;
        }

        targetOffset =  targetOffset / 180.0 * Math.PI;

        // When the angle is off by more that 10 degrees, only rotate.
        if(Math.abs(targetOffset) > (10.0 / 180 * Math.PI)) {
            if(desiredAngle > currentAngle) {
                maxAngularRate = -maxAngularRate * shortestPathOverride;
            }
            throttleFactor = 0.5;
            return ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, throttleFactor * maxAngularRate, new Rotation2d(targetOffset));
        } else if(Math.abs(targetOffset) > (2.0 / 180 * Math.PI)) {
            if(desiredAngle > currentAngle) {
                maxAngularRate = -maxAngularRate * shortestPathOverride;
            }
            throttleFactor = 0.25;
            return ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, throttleFactor * maxAngularRate, new Rotation2d(targetOffset));
        } else {
            maxAngularRate = 0;
            
            range = m_fbsys.getLastArmRange();
            if(range > 450) {
                throttleFactor = 0.1;
            } else {
                throttleFactor = 0.0;
            }
            return ChassisSpeeds.fromFieldRelativeSpeeds(throttleFactor, 0.0, 0.0, new Rotation2d(0.0));
        }

    }

}
