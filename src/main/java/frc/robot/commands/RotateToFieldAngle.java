package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavSubsystem;

public class RotateToFieldAngle extends Command {
    private double m_desiredAngle;
    double m_maxAngularRate;
    DriveSubsystem m_driveSubsystem;

    public RotateToFieldAngle(DriveSubsystem driveSubsystem, double angle) {
        m_desiredAngle = angle;
        m_maxAngularRate = (90.0 - 20.0) / 180.0 * Math.PI;  // 70 deg/sec in radians
        m_driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double currentAngle;
        double targetOffset;
        double shortestPathOverride;
        double throttleFactor;

        // Determine the angle offset and correct that first
        currentAngle = NavSubsystem.getContinuousAngle();

        double revs = Math.floor(currentAngle / 360.0);
        currentAngle = currentAngle - (revs * 360.0);

        targetOffset = (m_desiredAngle - currentAngle);

        shortestPathOverride = 1.0;
        if(targetOffset > 180.0) {
            shortestPathOverride = -1.0;
            targetOffset -= 180.0;
        }

        targetOffset =  targetOffset / 180.0 * Math.PI;

        // When the angle is off by more that 10 degrees, only rotate.
        if(Math.abs(targetOffset) > (10.0 / 180 * Math.PI)) {
            if(m_desiredAngle > currentAngle) {
                m_maxAngularRate = -m_maxAngularRate * shortestPathOverride;
            }
            throttleFactor = 0.5;
            m_driveSubsystem.setDesiredChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, throttleFactor * m_maxAngularRate, new Rotation2d(targetOffset)));
        } else if(Math.abs(targetOffset) > (2.0 / 180 * Math.PI)) {
            if(m_desiredAngle > currentAngle) {
                m_maxAngularRate = -m_maxAngularRate * shortestPathOverride;
            }
            throttleFactor = 0.25;
            m_driveSubsystem.setDesiredChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, throttleFactor * m_maxAngularRate, new Rotation2d(targetOffset)));
        }
    }

    @Override
    public boolean isFinished() {
        double currentAngle;
        double targetOffset;

        // Determine the angle offset and correct that first
        currentAngle = NavSubsystem.getLocalAngle();

        targetOffset = (m_desiredAngle - currentAngle);
        if(targetOffset > 180.0) {
            targetOffset -= 180.0;
        }

        targetOffset =  targetOffset / 180.0 * Math.PI;

        if(Math.abs(targetOffset) <= (2.0 / 180 * Math.PI)) {
            m_driveSubsystem.setDesiredChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, new Rotation2d(0.0)));
        } else {
            return false;
        }

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            m_driveSubsystem.setDesiredChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, new Rotation2d(0.0)));
        }
    }
    
}
