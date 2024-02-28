package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GoForwardAndBack extends SequentialCommandGroup {
    public GoForwardAndBack(Trajectories trajectories) {
        addCommands(
            trajectories.driveTrajectory(trajectories.PullForwardAndBackTrajectory), 
            new WaitCommand(3.0), 
            trajectories.driveTrajectory(trajectories.PullForwardAndBackTrajectory2));
    }
}
