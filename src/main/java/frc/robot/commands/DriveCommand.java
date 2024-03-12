// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FrontBackUltrasonicSubsystem;
import frc.robot.subsystems.NavSubsystem;
//import frc.robot.tasks.MoveToUltrasonicPositionTask;

import java.util.Date;

public class DriveCommand extends Command {
  private DriveSubsystem m_driveSubsystem;
  private NavSubsystem m_NavSubsystem;
  private DriveController m_dc;
  private long m_lastUpdate;
  private final int MOTOR_UPDATE_FREQUENCY = 10;  // msec
  private SwerveModuleState desiredMovement;
  private int simControl;
  private int i;
  private FrontBackUltrasonicSubsystem m_fbsys;
  //private MoveToUltrasonicPositionTask forwardStationTask;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveController dc, DriveSubsystem driveSubsystem, FrontBackUltrasonicSubsystem fbsys) {
    m_dc = dc;
    m_driveSubsystem = driveSubsystem;
    m_fbsys = fbsys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dc);
    addRequirements(m_driveSubsystem);
    desiredMovement = new SwerveModuleState(0.0, new Rotation2d(0.0));
    simControl = 0;
    i=0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle;
    double throttle;

    Rotation2d ang;
    
    angle = m_dc.getDesiredAngle();

    throttle = m_dc.getDesiredThrottle() * 1.0;
    
    if(!m_dc.overrideAutoThrottle()) {
      if(m_fbsys != null) {
        SmartDashboard.putNumber("Ramp Range", m_fbsys.getLastRampRange());
        SmartDashboard.putNumber("Arm Range", m_fbsys.getLastArmRange());
        if((m_fbsys.getLastRampRange() > 0.0 && m_fbsys.getLastRampRange() < 1000.0) || 
           (m_fbsys.getLastArmRange() > 0.0 && m_fbsys.getLastArmRange() < 1000.0)) {
          throttle = throttle / Constants.MAX_VELOCITY * Constants.SLOW_VELOCITY;
        }
      } 
    }

    SmartDashboard.putNumber("DesiredThrottle (m/s)", throttle);
    
    ang = new Rotation2d(m_dc.getDesiredVX(), m_dc.getDesiredVY());
    desiredMovement = new SwerveModuleState(throttle, ang);
    SmartDashboard.putNumber("Desired Angle (deg)", ang.getDegrees());
    
    m_driveSubsystem.setDesiredSwerveState(desiredMovement, m_dc.getRotation(), NavSubsystem.getContinuousAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /*public void setMoveToForwardStationTask(MoveToUltrasonicPositionTask task) {
    forwardStationTask = task;
  }*/
}
