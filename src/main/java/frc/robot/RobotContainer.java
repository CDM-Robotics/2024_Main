package frc.robot;

import frc.robot.commands.*;
import frc.robot.devices.SwerveAssembly;
import frc.robot.devices.NavX;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;

public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  private SwerveAssembly frontLeft;
  private SwerveAssembly frontRight;
  private SwerveAssembly rearLeft;
  private SwerveAssembly rearRight;
  private DriveController m_DriveController;
  private DriveSubsystem m_DriveSubsystem;
  private DrivePhysics m_DrivePhysics;
  private UpdateSwerveStateCommand m_updateSwerveCommand;
  private DriveCommand m_DriveCommand;
  private ArrayList<SwerveAssembly> swerves;
  private NavX navx;
  private NavSubsystem m_navSubsystem;
  private NavCommand m_navCommand;
  
  private RobotContainer() {
        // Setup the navigation sensor
        navx = new NavX();
        m_navSubsystem = new NavSubsystem(navx);
        m_navCommand = new NavCommand(m_navSubsystem);
        m_navSubsystem.setDefaultCommand(m_navCommand);

        // Define the 4 Swerve Assemblies
        frontLeft = new SwerveAssembly("Front Left", 2, 3, false, Constants.WHEEL_OFFSET_X, Constants.WHEEL_OFFSET_Y, Constants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
        frontRight = new SwerveAssembly("Front Right", 4, 5, false, Constants.WHEEL_OFFSET_X, -Constants.WHEEL_OFFSET_Y, Constants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
        rearLeft = new SwerveAssembly("Rear Left", 8, 9, false, -Constants.WHEEL_OFFSET_X, Constants.WHEEL_OFFSET_Y,Constants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);
        rearRight = new SwerveAssembly("Rear Right", 6, 7, false, -Constants.WHEEL_OFFSET_X, -Constants.WHEEL_OFFSET_Y, Constants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);
        swerves = new ArrayList<SwerveAssembly>();
        swerves.add(frontLeft);
        swerves.add(frontRight);
        swerves.add(rearLeft);
        swerves.add(rearRight);

        // Add the Swerve Assemblies to the DrivePhysics Subsystem and set the default (UpdateSwerveState) command
        m_DrivePhysics = new DrivePhysics(frontLeft, frontRight, rearLeft, rearRight);
        m_updateSwerveCommand = new UpdateSwerveStateCommand(m_DrivePhysics);
        m_DrivePhysics.setDefaultCommand(m_updateSwerveCommand);

        m_DriveController = new DriveController();
        m_DriveSubsystem = new DriveSubsystem(m_DrivePhysics, frontLeft, frontRight, rearLeft, rearRight);

        m_DriveCommand = new DriveCommand(m_DriveController, m_DriveSubsystem);
        m_DriveController.setDefaultCommand(m_DriveCommand);        
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return null;
  }

  public Boolean initializeDriveSubsystem() {
    return m_DriveSubsystem.initialize();
  }
  
}

