// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavSubsystem;

public class DriveStraightCommand extends Command {
  private DriveSubsystem m_driveSubsystem;
  private DriveController m_dc;
  private SwerveModuleState desiredMovement;
  private double m_distance;
  private Pose2d m_currentPose;
  private Pose2d m_endPose;
  private double m_cruiseVelocity;
  private double m_slowVelocity;
  private double m_endFieldAngle;
  private Pose2d m_initialPose;
  private Translation2d initialTranslation;
  private Translation2d endTranslation;
  private Translation2d translationDifference;
  private double heading;
  private boolean m_fieldAlignmentComplete;
  static int inited;
  private final double POSE_THRESHOLD_INCHES = 6.0;

  static {
    inited = 0;
  }

  /** Creates a new DriveCommand. */
  public DriveStraightCommand(DriveSubsystem driveSubsystem, double distance) {
    m_driveSubsystem = driveSubsystem;
    m_distance = distance;
    m_currentPose = null;
    m_endPose = null;
    m_fieldAlignmentComplete = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  public DriveStraightCommand(DriveSubsystem driveSubsystem, Pose2d endPose, double cruiseVelocity, double slowVelocity, double endFieldAngle) {
    m_driveSubsystem = driveSubsystem;

    m_endPose = endPose;
    m_distance = 0.0;
    m_cruiseVelocity = cruiseVelocity;
    m_slowVelocity = slowVelocity;
    m_endFieldAngle = endFieldAngle;
    m_fieldAlignmentComplete = false;
    
    addRequirements(m_driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveStraightCommand.inited++;
    SmartDashboard.putNumber(("Drive Straight Inited"), inited);
    m_currentPose = m_driveSubsystem.getPose();
    m_initialPose = m_driveSubsystem.getPose();
    initialTranslation = m_initialPose.getTranslation();
    if(m_endPose != null) {
      endTranslation = m_endPose.getTranslation();
      translationDifference = endTranslation.minus(initialTranslation);
      heading = Math.atan2(translationDifference.getY(), translationDifference.getX());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_endPose == null) {
      driveToDistanceX();
    } else {
      driveToPose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
<<<<<<< HEAD
    if(m_endPose == null) {
      if(m_driveSubsystem.getPose().getX() >= m_distance) {
        m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0, 0.0)), 0.0, NavSubsystem.getContinuousAngle());
        return true;
      }
    } else {
      SmartDashboard.putNumber("Auto Pose Distance", Units.metersToInches(m_currentPose.getTranslation().getDistance(m_endPose.getTranslation())));
      if((m_currentPose.getTranslation().getDistance(m_endPose.getTranslation()) <= Units.inchesToMeters(POSE_THRESHOLD_INCHES))) {
        //m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0, 0.0)), 0.0, NavSubsystem.getContinuousAngle());
        return m_fieldAlignmentComplete;
      }
=======
    SmartDashboard.putNumber("Auto Pose X (Feet)", Units.metersToFeet(m_driveSubsystem.getPose().getX()));
    SmartDashboard.putNumber("Desired Auto Pose X (Feet)", Units.metersToFeet(m_distance));
    if(m_driveSubsystem.getPose().getX() >= m_distance) {
      m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0, 0.0)), 0.0, NavSubsystem.getContinuousAngle());
      return true;
>>>>>>> 71a04ab9e611131c0e393baa2b989be44e41245f
    }

    return false;
  }

  private void driveToDistanceX() {
    double throttle;

    Rotation2d ang;

    throttle = Constants.SLOW_VELOCITY;

    ang = new Rotation2d(0.0, 0.0);
    desiredMovement = new SwerveModuleState(throttle, ang);
    SmartDashboard.putNumber("DesiredThrottle (m/s)", throttle);
    SmartDashboard.putNumber("Desired Angle (deg)", ang.getDegrees());
    
    m_driveSubsystem.setDesiredSwerveState(desiredMovement, 0.0, NavSubsystem.getContinuousAngle());
  }

  private void driveToPose() {
    double throttle = m_cruiseVelocity;
    Rotation2d ang;

    m_currentPose = m_driveSubsystem.getPose();
    /*Translation2d currentTranslation = m_currentPose.getTranslation();
    endTranslation = m_endPose.getTranslation();
    Translation2d desiredDifference = endTranslation.minus(currentTranslation);
    double desiredHeading = Math.atan2(desiredDifference.getY(), desiredDifference.getX());
    SmartDashboard.putNumber("Desired Heading", Units.metersToInches(desiredHeading));*/

    ang = new Rotation2d(heading);
    double distanceRemaining = m_currentPose.getTranslation().getDistance(m_endPose.getTranslation());
    if(distanceRemaining > Units.inchesToMeters(6) && distanceRemaining < Units.inchesToMeters(12)) {
      throttle = m_slowVelocity;
    } else if(distanceRemaining <= Units.inchesToMeters(POSE_THRESHOLD_INCHES)) {
      throttle = 0.0;
    }

    desiredMovement = new SwerveModuleState(throttle, ang /*new Rotation2d(0.0)*/);

    SmartDashboard.putNumber("DesiredThrottle (m/s)", throttle);
    SmartDashboard.putNumber("Desired Angle (deg)", ang.getDegrees());
    SmartDashboard.putNumber("Distance Remaining", Units.metersToInches(distanceRemaining));

    DriveAlignToAngle cmd = new DriveAlignToAngle(m_driveSubsystem, m_endFieldAngle);
    SmartDashboard.putBoolean("Destination Reached", false);
    if(cmd.isAligned()) {
      SmartDashboard.putBoolean("Destination Reached", true);
      m_fieldAlignmentComplete = true;
    } else {
      cmd.schedule();
    }

    m_driveSubsystem.setDesiredSwerveState(desiredMovement, 0.0, NavSubsystem.getContinuousAngle());;
  }
}
