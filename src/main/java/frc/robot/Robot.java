// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Robot.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

/*import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;*/

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    private Command m_testCommand;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    /*private final LoggedDashboardChooser<String> chooser =
      new LoggedDashboardChooser<>("Auto Choices");*/
    private static final String defaultAuto = "Default";
    private static final String customAuto = "My Auto";
    private int count;
    private final SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

    private Optional<DriverStation.Alliance> alliance;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        count = 0;

        try {
            Thread.sleep(3000);
        } catch(Exception e) {
            System.out.println("Error while sleeping.......");
            e.printStackTrace();
        } finally {
            System.out.println("Sleep complete ;-)");
        }

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        boolean logData = SmartDashboard.getBoolean("Log Drive Data", false);
        SmartDashboard.putBoolean("Log Drive Data", logData);

        DriverStation.waitForDsConnection(0.0);
        // Get the alliance information and post it to the SmartDashboard
        alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
        SmartDashboard.putString("ALLIANCE", alliance.get().name());
        SmartDashboard.putStringArray("POSITION", new String[]{"Red 1", "Red 2", "Red 3"});
        } else {
        SmartDashboard.putString("ALLIANCE", "!!!WARNING, NOT SET!!!");
        }

        SmartDashboard.putData("AUTO", m_Chooser);

        m_Chooser.addOption("Go Forward Only", m_robotContainer.auto_goForwardOnly);
        //m_Chooser.addOption("Simple Path", m_robotContainer.auto_simplePath);

        m_Chooser.setDefaultOption("Simple Path", m_robotContainer.auto_simplePath);

        
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
    }

    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }


    /**
    * This function is called once each time the robot enters Disabled mode.
    */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
    * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit() {
        SmartDashboard.putBoolean("TestMode", false);

        if(SmartDashboard.getString("ALLIANCE","WARNING").contains("WARNING")) {
            // Get the alliance information and post it to the SmartDashboard
            alliance = DriverStation.getAlliance();
            if(alliance.isPresent()) {
            SmartDashboard.putString("ALLIANCE", alliance.get().name());
            SmartDashboard.putStringArray("POSITION", new String[]{"Red 1", "Red 2", "Red 3"});
            } else {
            SmartDashboard.putString("ALLIANCE", "!!!WARNING, NOT SET!!!");
            }
        }
        
        
        m_autonomousCommand = m_Chooser.getSelected();
        //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {
        count++;
        //if(!m_autonomousCommand.isFinished()) {
        //    m_autonomousCommand.execute();
        //}
        SmartDashboard.putNumber("AUTONOMOUS", count);
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putBoolean("TeleopMode", false);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        m_robotContainer.enableDriveCommand();
        m_robotContainer.enableEngineeringCommand();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void testInit() {
        SmartDashboard.putBoolean("TestMode", true);
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        m_testCommand = m_robotContainer.getTestCommand();
        if(m_testCommand != null) {
            m_testCommand.schedule();
        }
    }

    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
        
    }
}