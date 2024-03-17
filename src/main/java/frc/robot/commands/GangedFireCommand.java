package frc.robot.commands;

import frc.robot.subsystems.GangedMotorSubsystem;

import java.time.LocalTime;
import java.util.Date;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class GangedFireCommand extends Command {
    GangedMotorSubsystem m_gangedMotorSybsystem;
    boolean m_onoff;
    double velocity;
    double reverseVelocity;

    public GangedFireCommand(GangedMotorSubsystem gangedMotorSubsystem, boolean onoff) {
        m_gangedMotorSybsystem = gangedMotorSubsystem;
        m_onoff = onoff;
    }

    @Override
    public void execute() {
        velocity = SmartDashboard.getNumber("Motor Fire Speed", velocity);
        reverseVelocity = SmartDashboard.getNumber("Motor Load Speed", reverseVelocity);

        if(m_gangedMotorSybsystem != null) {
            if(m_onoff) {
                m_gangedMotorSybsystem.setVelocity(velocity * 1.0); // 35.0
                System.out.print(".");
                System.out.flush();
            } else {
                m_gangedMotorSybsystem.setVelocity(0.0);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Run indefinately when turned on, it will need to be interrupted by a ParallelDeadlineGroup/WaitCommand
        return !m_onoff;
    }

    @Override
    public void end(boolean interrupted) {
        LocalTime lt = java.time.LocalTime.now();
        System.out.println("Ganged " + ((m_onoff) ? "ON" : "OFF") + " command finished at time: " + lt);
    }
}
