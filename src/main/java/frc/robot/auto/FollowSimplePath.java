package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveAlignToAngle;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.EngineerCommand;
import frc.robot.commands.GangedFireCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GangedMotorSubsystem;

public class FollowSimplePath extends SequentialCommandGroup {
    public FollowSimplePath(DriveSubsystem driveSubsystem, GangedMotorSubsystem gc, Trajectories trajectories) {
        Pose2d firstPose = new Pose2d(new Translation2d(Units.inchesToMeters(3.0 * 12), Units.inchesToMeters(0.0)), new Rotation2d(0.0));
        Pose2d secondPose = new Pose2d(new Translation2d(Units.inchesToMeters(3.0 * 12), Units.inchesToMeters(-12.0)), new Rotation2d(0.0));
        Pose2d thirdPose = new Pose2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.0)), new Rotation2d(0.0));
        
        addCommands(
            new DriveStraightCommand(driveSubsystem, firstPose, Constants.MAX_VELOCITY, Constants.SLOW_VELOCITY, 180.0),
            new DriveStraightCommand(driveSubsystem, secondPose, Constants.MAX_VELOCITY, Constants.SLOW_VELOCITY, 180.0),
            new DriveStraightCommand(driveSubsystem, thirdPose, Constants.MAX_VELOCITY, Constants.SLOW_VELOCITY, 180.0)

            //new ResetOdometryCommand(driveSubsystem, trajectories.PullForwardTrajectory),
           //trajectories.driveTrajectory(trajectories.PullForwardTrajectory) 
            //new ResetOdometryCommand(driveSubsystem, trajectories.PullBackToStartTrajectory),
            /*trajectories.driveTrajectory(trajectories.PullBackToStartTrajectory),
            new ParallelDeadlineGroup(new WaitCommand(1.5), new GangedFireCommand(gc, true) ),
            new ParallelCommandGroup(new WaitCommand(0.5), new GangedFireCommand(gc, false))*/
            );
    }
}
