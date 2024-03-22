package frc.robot.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.EngineerCommand;
import frc.robot.commands.GangedFireCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GangedMotorSubsystem;

public class GoForwardAndBack extends SequentialCommandGroup {
    public GoForwardAndBack(DriveSubsystem driveSubsystem, GangedMotorSubsystem gc, Trajectories trajectories) {
        addCommands(
            new DriveStraightCommand(driveSubsystem, Units.inchesToMeters(36.0))
            //new ResetOdometryCommand(driveSubsystem, trajectories.PullForwardTrajectory),
           //trajectories.driveTrajectory(trajectories.PullForwardTrajectory) 
            //new ResetOdometryCommand(driveSubsystem, trajectories.PullBackToStartTrajectory),
            /*trajectories.driveTrajectory(trajectories.PullBackToStartTrajectory),
            new ParallelDeadlineGroup(new WaitCommand(1.5), new GangedFireCommand(gc, true) ),
            new ParallelCommandGroup(new WaitCommand(0.5), new GangedFireCommand(gc, false))*/
            );
    }
}
