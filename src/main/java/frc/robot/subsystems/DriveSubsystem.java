// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.devices.SwerveAssembly;
import frc.robot.exceptions.MotorSetupException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
    
    public DrivePhysics m_physics;
    private SwerveDriveKinematics kinematics;
    private SwerveAssembly[] m_assemblies;
    

    public DriveSubsystem(DrivePhysics physics, SwerveAssembly... assemblies) {
        if(assemblies.length != 4) {
            return;
        }
        
        Translation2d[] locations = {assemblies[0].location, assemblies[1].location, assemblies[2].location, assemblies[3].location};
        m_physics = physics;
        
        kinematics = new SwerveDriveKinematics(locations);
        m_assemblies = assemblies;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Log empty setpoint states when disabled
        /*if (DriverStation.isDisabled()) {
            Logger.getInstance().recordOutput("Odometry", new SwerveModuleState[] {});
        }*/
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public Boolean initialize() {
        try {
            for (SwerveAssembly assembly : m_assemblies) {
                assembly.initialize();
            }
        } catch(MotorSetupException mse) {
            System.out.println(mse.getMessage());
            return false;
        }

        return true;
    }

    public void setDesiredSwerveState(SwerveModuleState state, double rotation, double fieldAngle) {
        double vx, vy;

        SmartDashboard.putNumber("Desired Angle (x)", state.angle.getCos());
        SmartDashboard.putNumber("Desired Angle (y)", state.angle.getSin());

        SwerveModuleState allowed = m_physics.askPermissionToMove(state);

        vx = allowed.speedMetersPerSecond * allowed.angle.getCos();
        vy = allowed.speedMetersPerSecond * allowed.angle.getSin();

        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotation, Rotation2d.fromDegrees(-fieldAngle));
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassis);

        // Log setpoint states
        //Logger.getInstance().recordOutput("Odometry", states);

        // If we are translating and rotating at the same time, we need to make sure one of the wheels doesn't
        // get commanded to travel too fast
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.1 /*Constants.MAX_WHEEL_VELOCITY*/);

        m_assemblies[0].setState(states[0]);
        m_assemblies[1].setState(states[1]);
        m_assemblies[2].setState(states[2]);
        m_assemblies[3].setState(states[3]);

        
    }

    public void setRotation(double rotation) {
        double pureRotationSpeed = 0.5;
    }
}
