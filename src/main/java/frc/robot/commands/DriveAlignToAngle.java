package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavSubsystem;

public class DriveAlignToAngle extends Command {
    private double m_desiredAngle;
    double m_maxAngularRate;
    DriveSubsystem m_driveSubsystem;

    public DriveAlignToAngle(DriveSubsystem driveSubsystem) {
        m_desiredAngle = 0.0;
        m_maxAngularRate = (90.0 - 20.0) / 180.0 * Math.PI;  // 70 deg/sec in radians
        m_driveSubsystem = driveSubsystem;
    }

    public DriveAlignToAngle(DriveSubsystem driveSubsystem, double desiredAngle) {
        m_desiredAngle = desiredAngle;
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
        double omega;

        SmartDashboard.putNumber("Drive Align-to-Angle", m_desiredAngle);

        // Determine the angle offset and correct that first
        currentAngle = NavSubsystem.getRawAngle();

        double revs = Math.floor(currentAngle / 360.0);
        currentAngle = currentAngle - (revs * 360.0);

        targetOffset = (m_desiredAngle - currentAngle);

        shortestPathOverride = 1.0;
        if(Math.abs(targetOffset) > 180.0) {
            shortestPathOverride = -1.0;
            targetOffset -= 180.0;
        }

        targetOffset =  targetOffset / 180.0 * Math.PI;

        if(targetOffset > 0) {
            omega = -m_maxAngularRate * shortestPathOverride;
        } else {
            omega = m_maxAngularRate * shortestPathOverride;
        }

        // When the angle is off by more that 10 degrees, only rotate.
        if(Math.abs(targetOffset) > (10.0 / 180 * Math.PI)) {
            SmartDashboard.putString("Drive Align-to-Angle Status", "Turn Fast");
            throttleFactor = 0.75;
            m_driveSubsystem.setFieldAlignment(throttleFactor * omega);
        } else if(Math.abs(targetOffset) > (2.0 / 180 * Math.PI)) {
            SmartDashboard.putString("Drive Align-to-Angle Status", "Turn Slow");
            throttleFactor = 0.25;
            m_driveSubsystem.setFieldAlignment(throttleFactor * omega);
        } else {
            SmartDashboard.putString("Drive Align-to-Angle Status", "Stop Turning");
            m_driveSubsystem.setFieldAlignment(0.0);
        }
    }

    public boolean isAligned() {
        double currentAngle;
        double targetOffset;
        double shortestPathOverride;
        double throttleFactor;
        double omega;

        SmartDashboard.putNumber("Drive Align-to-Angle", m_desiredAngle);

        // Determine the angle offset and correct that first
        currentAngle = NavSubsystem.getRawAngle();

        double revs = Math.floor(currentAngle / 360.0);
        currentAngle = currentAngle - (revs * 360.0);

        targetOffset = (m_desiredAngle - currentAngle);

        shortestPathOverride = 1.0;
        if(Math.abs(targetOffset) > 180.0) {
            shortestPathOverride = -1.0;
            targetOffset -= 180.0;
        }

        targetOffset =  targetOffset / 180.0 * Math.PI;

        // When the angle is off by more that 10 degrees, only rotate.
        if(Math.abs(targetOffset) > (10.0 / 180 * Math.PI)) {
            return false;
        } else if(Math.abs(targetOffset) > (2.0 / 180 * Math.PI)) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    public void setAngle(double angle) {
        m_desiredAngle = angle;
    }
}
