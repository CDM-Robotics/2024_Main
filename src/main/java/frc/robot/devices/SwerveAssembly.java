// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.devices;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.exceptions.MotorSetupException;

/** Add your docs here. */
public class SwerveAssembly {

    public final double STEERING_RATIO = 150.0 / 7.1;
    //public final double DRIVE_RATIO = 6.75;
    int encoderCtr;
    //public final double STEERING_RATIO = 1.0;
    public final double DRIVE_RATIO = 1.0;

    private SteeringMotor m_steeringMotor;
    private DrivingMotor m_driveMotor;
    private String m_prettyName;
    public Translation2d location;
    private double m_chassisAngularOffset;

    public SwerveAssembly(String pretty, int steeringID, int driveID, boolean driveInverted, double posx, double posy, double chassisAngularOffset) {
        m_steeringMotor = new SteeringMotor(steeringID, false);

        m_driveMotor = new DrivingMotor(driveID);

        m_prettyName = pretty;
        m_driveMotor.setInverted(driveInverted);
        location = new Translation2d(posx, posy);
        m_chassisAngularOffset = chassisAngularOffset;
    }

    public void initialize() throws MotorSetupException {
        m_steeringMotor.initialize();
        m_steeringMotor.setAngle(0.0);
        m_driveMotor.initialize();
        m_driveMotor.setVelocity(0.0);
    }

    public void setState(SwerveModuleState s) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = s.speedMetersPerSecond;
        correctedDesiredState.angle = s.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, m_steeringMotor.getAngle());

        m_steeringMotor.setAngle(optimizedDesiredState.angle);
        m_driveMotor.setVelocity(optimizedDesiredState.speedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getVelocity(), m_steeringMotor.getAngle());
    }

    public String getPrettyName() {
        return m_prettyName;
    }

    public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_driveMotor.getPosition(),
        new Rotation2d(m_steeringMotor.getPosition() - m_chassisAngularOffset));
  }
}
