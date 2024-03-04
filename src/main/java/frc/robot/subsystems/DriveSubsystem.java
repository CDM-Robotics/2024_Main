// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveAlignToAngle;
import frc.robot.devices.SwerveAssembly;
import frc.robot.exceptions.MotorSetupException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
// Odometry
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
    
    public DrivePhysics m_physics;
    private SwerveDriveKinematics kinematics;
    private SwerveAssembly[] m_assemblies;
    private SwerveAssembly frontLeft;
    private SwerveAssembly frontRight;
    private SwerveAssembly rearLeft;
    private SwerveAssembly rearRight;
    private NavSubsystem navSubsystem;
    private ChassisSpeeds m_fieldAlignment;

    SwerveDriveOdometry m_odometry = null;



    public DriveSubsystem(NavSubsystem navSys) {
        navSubsystem = navSys;
        frontLeft = new SwerveAssembly("Front Left", 9, 4, false, Constants.WHEEL_OFFSET_X, Constants.WHEEL_OFFSET_Y, Constants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
        frontRight = new SwerveAssembly("Front Right", 3, 8, false, Constants.WHEEL_OFFSET_X, -Constants.WHEEL_OFFSET_Y, Constants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
        rearLeft = new SwerveAssembly("Rear Left", 5, 2, false, -Constants.WHEEL_OFFSET_X, Constants.WHEEL_OFFSET_Y,Constants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);
        rearRight = new SwerveAssembly("Rear Right", 7, 6, false, -Constants.WHEEL_OFFSET_X, -Constants.WHEEL_OFFSET_Y, Constants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);
        m_assemblies = new SwerveAssembly[4];
        m_assemblies[0] = frontLeft;
        m_assemblies[1] = frontRight;
        m_assemblies[2] = rearLeft;
        m_assemblies[3] = rearRight;

        m_fieldAlignment = null;

        Translation2d[] locations = {m_assemblies[0].location, m_assemblies[1].location, m_assemblies[2].location, m_assemblies[3].location};
        kinematics = new SwerveDriveKinematics(locations);
        
        m_physics = new DrivePhysics(frontLeft, frontRight, rearLeft, rearRight);

        AutoBuilder.configureHolonomic(this::getPathPose, 
            this::resetOdometry, 
            this::getRobotRelativeSpeeds, 
            this::setDesiredChassisSpeeds, 
            Constants.kHolonomicPathFollowerConfig, 
            () -> {
                return false;
            }, 
            this);
    }

    public DrivePhysics getDrivePhysics() {
        return m_physics;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
            getAngle(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }
        );

        SmartDashboard.putNumber("Field Pos X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Field Pos Y", m_odometry.getPoseMeters().getY());

        m_physics.updateState();
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

            m_odometry = new SwerveDriveOdometry(
                Constants.kDriveKinematics,
                getAngle(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            });

        } catch(MotorSetupException mse) {
            System.out.println(mse.getMessage());
            return false;
        }

        return true;
    }

    public void setDesiredSwerveState(SwerveModuleState state, double rotation, double fieldAngle) {
        double vx, vy;

        SwerveModuleState allowed = m_physics.askPermissionToMove(state);

        vx = allowed.speedMetersPerSecond * allowed.angle.getCos();
        vy = allowed.speedMetersPerSecond * allowed.angle.getSin();

        ChassisSpeeds chassis;
        if(m_fieldAlignment != null) {
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, m_fieldAlignment.omegaRadiansPerSecond, Rotation2d.fromDegrees(-fieldAngle));
            m_fieldAlignment = null;  // Reset
            rotation = 0.0;
            chassis = chassis.plus(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotation, Rotation2d.fromDegrees(-fieldAngle)));
        } else {
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotation, Rotation2d.fromDegrees(-fieldAngle));
        }

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

    public void setDesiredChassisSpeeds(ChassisSpeeds chassis) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassis);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.1 /*Constants.MAX_WHEEL_VELOCITY*/);

        m_assemblies[0].setState(states[0]);
        m_assemblies[1].setState(states[1]);
        m_assemblies[2].setState(states[2]);
        m_assemblies[3].setState(states[3]);
    }

    public Pose2d getPose() {
        if(m_odometry != null) {
            return m_odometry.getPoseMeters();
        } else {
            return null;
        }
    }

    public Pose2d getPathPose() {
        return getPose().rotateBy(new Rotation2d(Units.degreesToRadians(90.0)));
    }

    public void resetOdometry(Pose2d pose) {
        if(m_odometry != null) {
            m_odometry.resetPosition(
                getAngle(),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
                },
                pose);
        }
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-NavSubsystem.getContinuousAngle());
    }

      /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_assemblies[0].setState(desiredStates[0]);
    m_assemblies[1].setState(desiredStates[1]);
    m_assemblies[2].setState(desiredStates[2]);
    m_assemblies[3].setState(desiredStates[3]);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(m_assemblies[0].getState(), 
        m_assemblies[1].getState(), 
        m_assemblies[2].getState(), 
        m_assemblies[3].getState());
  }

  public void setFieldAlignment(double omega) {
    m_fieldAlignment = new ChassisSpeeds(0.0,0.0,omega);
  }
}
