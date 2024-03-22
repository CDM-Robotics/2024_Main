// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavSubsystem;

public class DriveStraightCommand extends Command {
  private DriveSubsystem m_driveSubsystem;
  private DriveController m_dc;
  private SwerveModuleState desiredMovement;
  private double m_distance;

  /** Creates a new DriveCommand. */
  public DriveStraightCommand(DriveSubsystem driveSubsystem, double distance) {
    m_driveSubsystem = driveSubsystem;
    m_distance = distance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    desiredMovement = new SwerveModuleState(0.0, new Rotation2d(0.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle;

    Rotation2d ang;

    throttle = Constants.MAX_VELOCITY * 0.25;

    SmartDashboard.putNumber("DesiredThrottle (m/s)", throttle);
    
    ang = new Rotation2d(0.0, 0.0);
    desiredMovement = new SwerveModuleState(throttle, ang);
    SmartDashboard.putNumber("Desired Angle (deg)", ang.getDegrees());
    
    m_driveSubsystem.setDesiredSwerveState(desiredMovement, 0.0, NavSubsystem.getContinuousAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Auto Pose X", m_driveSubsystem.getPose().getX());
    SmartDashboard.putNumber("Desired Auto Pose X", m_distance);
    if(m_driveSubsystem.getPose().getX() >= m_distance) {
      m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0, 0.0)), 0.0, NavSubsystem.getContinuousAngle());
      return true;
    }

    return false;
  }
}
