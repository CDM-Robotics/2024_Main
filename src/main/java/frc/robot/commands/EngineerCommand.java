package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.devices.SnowBlower;
import frc.robot.devices.SteeringMotor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.EngineerController;
import frc.robot.subsystems.GangedMotorSubsystem;
import frc.robot.subsystems.POSITION;

public class EngineerCommand extends Command {
    private ArmSubsystem m_armSubsystem;
    private EngineerController m_engineerController;
    private GangedMotorSubsystem m_gangedSubsystem;
    private ConveyorSubsystem m_conveyorSubsystem;
    private double velocity = 35.0; // 60.0 might be ideal
    private double reverseVelocity = -3.0; // -3.0 might be ideal
    private double armPercentOut = 0.50;
    private double armPercentIn = -0.20;

    public EngineerCommand(EngineerController engineer, ArmSubsystem armSubsystem, GangedMotorSubsystem gangedSubsystem, ConveyorSubsystem conveyorSubsystem) {
        addRequirements(armSubsystem);
        addRequirements(gangedSubsystem);

        m_engineerController = engineer;
        m_armSubsystem = armSubsystem;
        m_gangedSubsystem = gangedSubsystem;
        m_conveyorSubsystem = conveyorSubsystem;
    }

    @Override
    public void execute() {
        velocity = SmartDashboard.getNumber("Motor Fire Speed", velocity);
        reverseVelocity = SmartDashboard.getNumber("Motor Load Speed", reverseVelocity);

        if(m_gangedSubsystem != null) {
            if(m_engineerController.speakerReadyToFire()) {
                m_gangedSubsystem.setVelocity(velocity * 1.0); // 35.0
            } else if(m_engineerController.speakerReadyToIntake()) {
                m_gangedSubsystem.setVelocity(reverseVelocity * 1.0); // -3.0
            } else {
                m_gangedSubsystem.setVelocity(0.0);
            }
        }

        if(m_armSubsystem != null) {
            m_armSubsystem.setPosition(m_engineerController.getDesiredPosition());
        }

        if(m_conveyorSubsystem != null) {
            if(m_engineerController.isArmActionReady()) {
                if(m_armSubsystem.getPosition() == POSITION.AMP) {
                    m_conveyorSubsystem.setVelocity(armPercentOut);
                } else if(m_armSubsystem.getPosition() == POSITION.SOURCE) {
                    m_conveyorSubsystem.setVelocity(armPercentIn);
                }
            } else {
                m_conveyorSubsystem.setVelocity(0.0);
            }
        }
    }
}       
