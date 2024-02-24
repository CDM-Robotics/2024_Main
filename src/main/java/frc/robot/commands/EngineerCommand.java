package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.devices.SnowBlower;
import frc.robot.devices.SteeringMotor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EngineerController;
import frc.robot.subsystems.GangedMotorSubsystem;

public class EngineerCommand extends Command {
    private ArmSubsystem m_armSubsystem;
    private EngineerController m_engineerController;
    private GangedMotorSubsystem m_gangedSubsystem;
    private double velocity = 35.0; // 60.0 might be ideal
    private double reverseVelocity = -3.0; // -3.0 might be ideal

    public EngineerCommand(EngineerController engineer, ArmSubsystem armSubsystem, GangedMotorSubsystem gangedSubsystem) {
        addRequirements(armSubsystem);
        addRequirements(gangedSubsystem);

        m_engineerController = engineer;
        m_armSubsystem = armSubsystem;
        m_gangedSubsystem = gangedSubsystem;
    }

    @Override
    public void execute() {
        velocity = SmartDashboard.getNumber("Motor Fire Speed", velocity);
        reverseVelocity = SmartDashboard.getNumber("Motor Load Speed", reverseVelocity);

        if(m_engineerController.speakerReadyToFire()) {
            m_gangedSubsystem.setVelocity(velocity * 1.0); // 35.0
        } else if(m_engineerController.speakerReadyToIntake()) {
            m_gangedSubsystem.setVelocity(reverseVelocity * 1.0); // -3.0
        } else {
            m_gangedSubsystem.setVelocity(0.0);
        }

        m_armSubsystem.setPosition(m_engineerController.getDesiredPosition());
    }
}       
