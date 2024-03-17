package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class Trajectories {
    private DriveSubsystem m_driveSubsystem;

    public Trajectory PullForwardTrajectory;
    public Trajectory PullBackToStartTrajectory;
    private int tcount;

    public Trajectories(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        tcount = 0;

        TrajectoryConfig SimplePullForwardConfig =
            new TrajectoryConfig(
                AutoConstants.kSimplePullForwardSpeed,
                AutoConstants.kSimplePullForwardAccel)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.kDriveKinematics)
                .setReversed(true);

        TrajectoryConfig SimplePullForwardConfigReversed =
            new TrajectoryConfig(
                AutoConstants.kSimplePullForwardSpeed,
                AutoConstants.kSimplePullForwardAccel)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.kDriveKinematics)
                .setReversed(true);

        PullForwardTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.0, 0.0, new Rotation2d(0)),
                List.of(new Translation2d(Units.inchesToMeters(18.0), Units.inchesToMeters(6.0))),
                //List.of(),
                new Pose2d(Units.inchesToMeters(36.0), 0.0, new Rotation2d(Units.degreesToRadians(0.0))),
                SimplePullForwardConfig);

        PullBackToStartTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(36.0), 0.0, new Rotation2d(0)),
                List.of(new Translation2d(Units.inchesToMeters(18.0), Units.inchesToMeters(6.0))),
                //List.of(),
                new Pose2d(Units.inchesToMeters(0.0), 0.0, new Rotation2d(Units.degreesToRadians(180.0))),
                SimplePullForwardConfigReversed);
    }

    public Command driveTrajectory(Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
            0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
            m_driveSubsystem::getPose, // Functional interface to feed supplier
            Constants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0), // Position controllers
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_driveSubsystem::setModuleStates,
            m_driveSubsystem);
        
            return swerveControllerCommand;
        // Run path following command, then stop at the end.
        //return swerveControllerCommand.andThen(() -> m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0, 0.0)), 0.0, NavSubsystem.getContinuousAngle()));
    }

    public PathPlannerPath simplePath = PathPlannerPath.fromPathFile("Simple Path");
}
