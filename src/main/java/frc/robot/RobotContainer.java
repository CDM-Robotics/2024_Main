package frc.robot;

import frc.robot.commands.*;
import frc.robot.devices.*;
import frc.robot.subsystems.*;
import frc.robot.tasks.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ArrayList;

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
  


  private RobotContainer() {

    // Setup the navigation sensor
    navx = new NavX();
    m_navSubsystem = new NavSubsystem(navx);
    m_navCommand = new NavCommand(m_navSubsystem);
    m_navSubsystem.setDefaultCommand(m_navCommand);

    m_DriveSubsystem = new DriveSubsystem(m_navSubsystem);
    m_DriveSubsystem.initialize();

    m_DriveController = new DriveController();
    m_DriveCommand = new DriveCommand(m_DriveController, m_DriveSubsystem);
    m_DriveController.setDefaultCommand(m_DriveCommand);

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
    armSubsystem.setDefaultCommand(engineerCommand);

    m_ultrasonicSubsystem = new FrontBackUltrasonicSubsystem(0, "/dev/serial/by-id/usb-wch.cn_USB_Dual_Serial_0123456789-if00", "/dev/serial/by-id/usb-wch.cn_USB_Dual_Serial_0123456789-if02");
    if(!m_ultrasonicSubsystem.init()) {
      m_DriveCommand.setMoveToForwardStationTask(new MoveToUltrasonicPositionTask(m_ultrasonicSubsystem));
      System.out.println("There was an error while trying to initialzie subsystem, proceed with caution.");
    } else {
      m_DriveCommand.setMoveToForwardStationTask(new MoveToUltrasonicPositionTask(m_ultrasonicSubsystem));
    }
    
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
    // The selected command will be run in autonomous
    return null;
  }

  public Command getTestCommand() {
    return new ArmInitializationCommand(armSubsystem);
  }
  
}

