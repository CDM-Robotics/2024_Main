package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NavSubsystem;

public class NavCommand extends Command {
    private NavSubsystem m_NavSubsystem;
    private int dashUpdate;
    private double m_lastUpdate;

    public NavCommand(NavSubsystem nav) {
        dashUpdate = 1;
        m_NavSubsystem = nav;

        addRequirements(m_NavSubsystem);
    }

    @Override
    public void execute() {
        m_NavSubsystem.update();

        synchronized (this) {
            m_lastUpdate = NavSubsystem.getRawAngle();
        }

        if(dashUpdate%50 == 0) {
            SmartDashboard.putNumber("Field Angle", m_lastUpdate);
            dashUpdate = 1;
        }
        dashUpdate++;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
}
