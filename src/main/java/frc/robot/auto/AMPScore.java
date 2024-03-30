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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GangedMotorSubsystem;
import frc.robot.subsystems.POSITION;

public class AMPScore extends SequentialCommandGroup {
    public AMPScore(boolean yinverted, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, EngineerCommand ecmd, GangedMotorSubsystem gc, Trajectories trajectories) {
        double invert = 1.0;
        double endAngle = 90.0;

        addRequirements(driveSubsystem, armSubsystem);

        if(yinverted) {
            invert = -1.0;
            endAngle = 270.0;
        }
        this.setName("BLUE AMP Score");

        Pose2d firstPose = new Pose2d(new Translation2d(Units.inchesToMeters((72.5 -35.0) / 2.0), Units.inchesToMeters(invert * (63.5 - 17.5) / 2.0)), new Rotation2d(0.0));
        Pose2d secondPose = new Pose2d(new Translation2d(Units.inchesToMeters((72.5 - 35.0)), Units.inchesToMeters(invert * (63.5 - 17.5))), new Rotation2d(0.0));
        Pose2d thirdPose = new Pose2d(new Translation2d(Units.inchesToMeters(21.0 * 12.0), Units.inchesToMeters(invert * (63.5 - 17 - 2))), new Rotation2d(0.0));
        InstantCommand doit = new InstantCommand(() -> {
            armSubsystem.setPosition(POSITION.AMP);
        });
        doit.addRequirements(armSubsystem);
        addCommands(
            new DriveStraightCommand("AMP Score Midpoint", driveSubsystem, firstPose, 1.5, Constants.SLOW_VELOCITY, endAngle),
            doit,
            new DriveStraightCommand("AMP Score at AMP", driveSubsystem, secondPose, 1.5, Constants.SLOW_VELOCITY, endAngle)/*,
            new DriveStraightCommand("AMP Score Drive Out", driveSubsystem, thirdPose, Constants.MAX_VELOCITY, Constants.SLOW_VELOCITY, 0.0)*/
        );
    }
}