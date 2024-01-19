package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.devices.SteeringMotor;
import frc.robot.subsystems.DriveController;
import java.util.Date;

public class OrientationCommand extends Command {
    private final DriveController m_dc;
    private final SteeringMotor m_m1;
    private long m_lastUpdate;
    private final int MOTOR_UPDATE_FREQUENCY = 4000;  // 4x a second
    private double skips;
    private double averageSkips;

    public OrientationCommand(DriveController dc, SteeringMotor m1) {
        m_dc = dc;
        m_m1 = m1;
        skips = 0.0;
        averageSkips = 0.0;
        addRequirements(m_dc);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override 
    public void execute() {
        long currTime;
        Date d = new Date();
        currTime = d.getTime();
        if((currTime - m_lastUpdate) >= MOTOR_UPDATE_FREQUENCY) {
            m_m1.setAngle(m_dc.getDesiredAngle());
            m_lastUpdate = currTime;
            averageSkips = (currTime - m_lastUpdate) / skips;
            skips=0.0; // Reset
        } else {
            skips = skips + 1.0;
        }
    }
}
