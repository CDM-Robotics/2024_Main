package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EngineerController;
import frc.robot.subsystems.POSITION;

public class ArmInitializationCommand extends Command {
    private ArmSubsystem m_armSubsystem;
    private EngineerController m_engineerController;

    public ArmInitializationCommand(ArmSubsystem armSubsystem) {
        addRequirements(armSubsystem);

        m_armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        m_armSubsystem.gotoStartPosition();
    }
}       
