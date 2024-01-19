// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.SwerveAssembly;
import java.lang.Math;
//import org.littletonrobotics.junction.Logger;

import javax.sql.rowset.serial.SerialArray;

public class DrivePhysics extends SubsystemBase {
  private final double ROBOT_MASS = 56.7; // 56.7 kilograms = 125 lbs

  private double m_centerOfMassHeight;
  private double m_centerOfMassToWorstCaseTipAxis;
  private double m_weightInNewtons;

  public double totalLinearMomentum;
  public double totalVelocity;
  public double directionOfTravel;
  private double overTurningMoment;
  private SwerveAssembly[] m_assemblies;
  private SwerveDriveKinematics m_kinematics;

  /** Creates a new DrivePhysics. */
  public DrivePhysics(SwerveAssembly... assemblies) {
    m_assemblies = assemblies;

    totalLinearMomentum = 0.0;
    totalVelocity = 0.0;
    directionOfTravel = 0.0;

    m_centerOfMassHeight = 16 * 0.0254;               // 16 inches in meters
    m_centerOfMassToWorstCaseTipAxis = 12 * 0.0254;   // 12 inches in meters
    m_weightInNewtons = ROBOT_MASS * 9.81;
    overTurningMoment = m_weightInNewtons * m_centerOfMassToWorstCaseTipAxis;
    m_kinematics = new SwerveDriveKinematics(assemblies[0].location, assemblies[1].location, assemblies[2].location, assemblies[3].location);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateState() {
    SwerveModuleState flState, frState, rlState, rrState;

    double speed;

    //Logger log = Logger.getInstance();

    speed = 0.0;

    flState = m_assemblies[0].getState();
    frState = m_assemblies[1].getState();
    rlState = m_assemblies[2].getState();
    rrState = m_assemblies[3].getState();

    /*log.recordOutput("Front Left Velocity", m_assemblies[0].getState().speedMetersPerSecond);
    log.recordOutput("Front Right Velocity", m_assemblies[1].getState().speedMetersPerSecond);
    log.recordOutput("Rear Left Velocity", m_assemblies[2].getState().speedMetersPerSecond);
    log.recordOutput("Rear Right Velocity", m_assemblies[3].getState().speedMetersPerSecond);*/

    // TODO - Also cap the maximum wheel speed incase there are erroneous measurements.
    // Set speed to 0 if the magnitude is less than 0.001
    if(Math.abs(m_assemblies[0].getState().speedMetersPerSecond) < 0.001) {
      flState = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }

    if(Math.abs(m_assemblies[1].getState().speedMetersPerSecond) < 0.001) {
      frState = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }
 
    if(Math.abs(m_assemblies[2].getState().speedMetersPerSecond) < 0.001) {
      rlState = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }
    
    if(Math.abs(m_assemblies[3].getState().speedMetersPerSecond) < 0.001) {
      rrState = new SwerveModuleState(0.0, new Rotation2d(0.0));
    }

    estimateRobotMomentum(flState, frState, rlState, rrState);
  }

  private void estimateRobotMomentum(SwerveModuleState... states) {
    //Logger log = Logger.getInstance();
    double rvx, rvy;

    /*ChassisSpeeds speed = m_kinematics.toChassisSpeeds(states);

    
    log.recordOutput("Total Velocity", totalVelocity);

    totalLinearMomentum = ROBOT_MASS * totalVelocity;
    log.recordOutput("Total Linear Momentum", totalLinearMomentum);

    directionOfTravel = Math.acos(speed.vxMetersPerSecond / totalVelocity) / Math.PI * 180.0;
    if(speed.vxMetersPerSecond < 0.0) {
      directionOfTravel = -directionOfTravel;
    }

    if(totalVelocity < 0.001) {
      directionOfTravel = 0.0;
    }

    return;*/

    // Determine the XY vector components for each wheel's direction-of-travel

    // The velocity components at the center of the base will be the Mean of the
    // respective velocity components

    rvx = (states[0].speedMetersPerSecond * states[0].angle.getCos() + 
           states[1].speedMetersPerSecond * states[1].angle.getCos() +
           states[2].speedMetersPerSecond * states[2].angle.getCos() +
           states[3].speedMetersPerSecond * states[3].angle.getCos()) / 4.0;
    
    rvy = (states[0].speedMetersPerSecond * states[0].angle.getSin() + 
           states[1].speedMetersPerSecond * states[1].angle.getSin() +
           states[2].speedMetersPerSecond * states[2].angle.getSin() +
           states[3].speedMetersPerSecond * states[3].angle.getSin()) / 4.0;

    totalVelocity = Math.sqrt(Math.pow(rvx, 2) + Math.pow(rvy, 2));
   // log.recordOutput("Total Velocity", totalVelocity);

    return;

//    SmartDashboard.putNumber("Physics: totalVelocity", totalVelocity);

    // Therefore, the total magnitude of the momentum is simply the total velocity * mass of the robot
//    totalLinearMomentum = ROBOT_MASS * totalVelocity;

    // And now... find the robots direction of travel???
//    if((totalVelocity >= 0.01) || (totalVelocity <= -0.01)) {
      // totalVelocity is only the magnitude, it has no direction
      // even though rvy may be negative, acos only returns values between 0 and PI,
      // so we have to check if we are on the negative x-axis, if so the directionOfTravel
      // needs to be multiplied by -1
//      directionOfTravel = Math.acos(rvy / totalVelocity) / Math.PI * 180.0;
//      if(rvx < 0.0) {
//        directionOfTravel = -directionOfTravel;
//      }
//    } else {
//      directionOfTravel = 0.0;
//    }
  }

  public SwerveModuleState askPermissionToMove(SwerveModuleState ask) {
    double rmx, rmy;
    double dvx, dvy;
    double cvx, cvy;
    double desiredAppliedForce;
    double allowedAppliedForce;
    double allowedMomentumChange;
    double allowedVelocityChange;
    double allowedVx;
    double allowedVy; 
    double permittedVelocity;
    double permittedAngle;

    return ask;

    // Sanity check the input and see if the velocity exceeds the Max Allowed
/*    if(Math.abs(ask.velocity) > Constants.MAX_VELOCITY) {
      if(ask.velocity > 0.0) {                                // BAD ROBOT!!!
        ask.velocity = Constants.MAX_VELOCITY;  
      } else {
        ask.velocity = -Constants.MAX_VELOCITY;
      }
    }*/

    // Find the individual components of the current momentum in X and Y
//    rmx = totalLinearMomentum * Math.sin(directionOfTravel / 180.0 * 3.1415);
//    rmy = totalLinearMomentum * Math.cos(directionOfTravel / 180.0 * 3.1415);

    // Given the desired direction of travel, find the max velocity change in that direction
/*    cvx = rmx / ROBOT_MASS;
    cvy = rmy / ROBOT_MASS;
    dvx = ask.vx - cvx;
    dvy = ask.vy - cvy;*/

    // Desired applied force at CoM
/*    desiredAppliedForce = Math.sqrt(Math.pow(dvx, 2) + Math.pow(dvy, 2)) * ROBOT_MASS / (Constants.MAX_TALON_CMD_RATE_MSEC / 1000.0);
    
    if(true) {
      return ask;
    }
    if(desiredAppliedForce * m_centerOfMassHeight * Constants.MARGIN_OF_SAFETY <= overTurningMoment) {
      return ask; // And ye shall receive
    }*/
    
    // Sadly, you're too greedy.  Let me put you in your place
//    allowedAppliedForce = overTurningMoment / Constants.MARGIN_OF_SAFETY / m_centerOfMassHeight;
//    allowedMomentumChange = allowedAppliedForce * (Constants.MAX_TALON_CMD_RATE_MSEC / 1000.0);
//    allowedVelocityChange = allowedMomentumChange / ROBOT_MASS;

    // The new velocity components are the current minus the allowed
//    allowedVx = totalVelocity * Math.sin(directionOfTravel / 180.0 * 3.1415) + allowedVelocityChange * dvx / Math.sqrt(Math.pow(dvx,2) + Math.pow(dvy,2));
//    allowedVy = totalVelocity * Math.cos(directionOfTravel / 180.0 * 3.1415) + allowedVelocityChange * dvy / Math.sqrt(Math.pow(dvx,2) + Math.pow(dvy,2));
//    permittedVelocity = Math.sqrt(Math.pow(allowedVx, 2) + Math.pow(allowedVy, 2));
//    permittedAngle = (Math.atan2(allowedVx, allowedVy)) / Math.PI * 180.0;

//    return new SwerveState(permittedAngle, permittedVelocity);
  }

  //public SwerveState getCurrentSwerveState() {
  //  return new SwerveState(directionOfTravel, totalVelocity);
  //}
}
