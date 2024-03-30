package frc.robot;

import frc.robot.auto.AMPScore;
import frc.robot.auto.FollowSimplePath;
import frc.robot.auto.GoForwardAndBack;
import frc.robot.auto.Trajectories;
import frc.robot.commands.*;
import frc.robot.devices.*;
import frc.robot.subsystems.*;
import frc.robot.tasks.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  private DriveController m_DriveController;
  private DriveSubsystem m_DriveSubsystem;
  private DriveCommand m_DriveCommand;
  private NavX navx;
  private NavSubsystem m_navSubsystem;
  private NavCommand m_navCommand;
  private EngineerController m_engineerController;
  private ArmSubsystem armSubsystem;
  private EngineerCommand engineerCommand;
  private GangedMotorSubsystem m_gangedSubsystem;
  private ConveyorSubsystem m_conveyorSubsystem;
  private FrontBackUltrasonicSubsystem m_ultrasonicSubsystem;
  private Trajectories trajectories;

  private DriveAlignToAngle m_alignToRampToSource;
  private DriveAlignToAngle m_alignToArmToSource;

  public Command auto_goForwardOnly;
  public Command auto_simplePath;
  public Command auto_ampScore;
  public Command auto_ampScoreInverted;


  private RobotContainer() {

    // Setup the navigation sensor
    navx = new NavX();
    m_navSubsystem = new NavSubsystem(navx);
    m_navCommand = new NavCommand(m_navSubsystem);
    m_navSubsystem.setDefaultCommand(m_navCommand);

    
    m_ultrasonicSubsystem = new FrontBackUltrasonicSubsystem(0, "/dev/serial/by-id/usb-wch.cn_USB_Dual_Serial_0123456789-if00", "/dev/serial/by-id/usb-wch.cn_USB_Dual_Serial_0123456789-if02");
    if(!m_ultrasonicSubsystem.init()) {
      m_ultrasonicSubsystem = null;
      System.out.println("There was an error while trying to initialzie subsystem, proceed with caution.");
    }

    m_DriveSubsystem = new DriveSubsystem(m_navSubsystem);
    
    if(m_DriveSubsystem.initialize()) {
      SmartDashboard.putString("Drive Subsystem", "INITIALIZED OK");
    } else {
      SmartDashboard.putString("Drive Subsystem", "!!!FAILED, RESTART CODE OR REBOOT!!!");
    }

    m_DriveController = new DriveController(new DriveAlignToAngle(m_DriveSubsystem));
    m_DriveCommand = new DriveCommand(m_DriveController, m_DriveSubsystem, m_ultrasonicSubsystem);

    //m_DriveSubsystem.setDefaultCommand(m_DriveCommand);
    m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()));

    // Set up Selectable Autonomous Commands


    if(Constants.payloadsEnabled) {
      m_gangedSubsystem = new GangedMotorSubsystem(10,11);
      if(m_gangedSubsystem.initialize()) {
        System.out.println("Ganged Motors Initialized");
      } else {
        System.out.println("Could not initialize ganged motors");
      }
          
      m_conveyorSubsystem = new ConveyorSubsystem(16);
      m_engineerController = new EngineerController();
      armSubsystem = new ArmSubsystem(m_engineerController);
      armSubsystem.initialize();
      armSubsystem.setPosition(POSITION.NONE);
      engineerCommand = new EngineerCommand(m_engineerController, armSubsystem, m_gangedSubsystem, m_conveyorSubsystem);
      
    }

    // Get ready for autonomous
    trajectories = new Trajectories(m_DriveSubsystem);

    // Create the autonomous selections
    auto_goForwardOnly = new GoForwardAndBack(m_DriveSubsystem, m_gangedSubsystem, trajectories);
    auto_simplePath = new FollowSimplePath(m_DriveSubsystem, m_gangedSubsystem, trajectories);
    auto_ampScore = new AMPScore(false, m_DriveSubsystem, m_gangedSubsystem, trajectories);
    auto_ampScoreInverted = new AMPScore(true, m_DriveSubsystem, m_gangedSubsystem, trajectories);
  }

  public void enableEngineeringCommand() {
    if(Constants.payloadsEnabled) {
      armSubsystem.setDefaultCommand(engineerCommand);
    }
  }

  public void enableDriveCommand() {
    m_DriveSubsystem.setDefaultCommand(m_DriveCommand);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    return new GoForwardAndBack(m_DriveSubsystem, m_gangedSubsystem, trajectories);

    // Save for 2025
    //m_DriveSubsystem.resetOdometry(new Pose2d(1, 6, new Rotation2d()));
    //return AutoBuilder.followPath(trajectories.simplePath);
  }

  public Command getTestCommand() {
    return new ArmInitializationCommand(armSubsystem);
  }
  
}

