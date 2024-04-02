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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
  private final double POSE_THRESHOLD_INCHES = 1.0;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setPoint;
  private TrapezoidProfile.State endState;
  private double m_lastTime;
  private String m_title;
  private double m_initialTime;

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
    m_title = null;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  public DriveStraightCommand(String title, DriveSubsystem driveSubsystem, Pose2d endPose, double cruiseVelocity, double slowVelocity, double endFieldAngle) {
    m_title = title;
    m_initialTime = Timer.getFPGATimestamp();
    m_driveSubsystem = driveSubsystem;
    m_endPose = endPose;
    m_distance = 0.0;
    m_cruiseVelocity = cruiseVelocity;
    m_slowVelocity = slowVelocity;
    m_endFieldAngle = endFieldAngle;
    m_fieldAlignmentComplete = false;
    profile = null;
    
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveStraightCommand.inited++;
    SmartDashboard.putNumber(("Drive Straight Inited"), inited);
    SmartDashboard.putNumber(m_title + " Drive Time", 0.0);
    m_currentPose = m_driveSubsystem.getPose();
    m_initialPose = m_driveSubsystem.getPose();
    initialTranslation = m_initialPose.getTranslation();
    if(m_endPose != null) {
      m_lastTime = Timer.getFPGATimestamp();
      endTranslation = m_endPose.getTranslation();
      translationDifference = endTranslation.minus(initialTranslation);
      heading = Math.atan2(translationDifference.getY(), translationDifference.getX());
    }

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      m_cruiseVelocity,
      m_cruiseVelocity);
      setPoint = new TrapezoidProfile.State(0, 0);
      endState = new TrapezoidProfile.State(m_initialPose.getTranslation().getDistance(m_endPose.getTranslation()), 0);
      profile = new TrapezoidProfile(constraints);
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
    double distanceTraveled;

    if(m_endPose == null) {
      if(m_driveSubsystem.getPose().getX() >= m_distance) {
        m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0, 0.0)), 0.0, NavSubsystem.getContinuousAngle());
        return true;
      }
    } else {
      if(RobotBase.isSimulation()) {
        m_endPose.rotateBy(new Rotation2d(Units.degreesToRadians(m_endFieldAngle)));
        m_driveSubsystem.resetOdometry(m_endPose);
        return true;
      }
      distanceTraveled = m_currentPose.getTranslation().getDistance(m_endPose.getTranslation());
      SmartDashboard.putNumber("Auto Pose Distance", Units.metersToInches(distanceTraveled));
      if(((setPoint.velocity < 0.01 && setPoint.velocity > -0.01) && (setPoint.position > (0.75 * distanceTraveled))) | (m_currentPose.getTranslation().getDistance(m_endPose.getTranslation()) <= Units.inchesToMeters(POSE_THRESHOLD_INCHES))) {
        if(m_fieldAlignmentComplete) {
          SmartDashboard.putNumber(m_title + " Drive Time", m_lastTime - m_initialTime);
          m_driveSubsystem.setDesiredSwerveState(new SwerveModuleState(0.0, new Rotation2d(0.0)), 0.0, 
            NavSubsystem.getContinuousAngle());;
        }
        return m_fieldAlignmentComplete;
      }
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
    if(profile != null) {
      double currentTime = Timer.getFPGATimestamp();
      setPoint = profile.calculate(currentTime - m_lastTime, setPoint, endState);
      m_lastTime = currentTime;
    }
    double throttle = setPoint.velocity;
    Rotation2d ang;

    m_currentPose = m_driveSubsystem.getPose();
    Translation2d currentTranslation = m_currentPose.getTranslation();
    Translation2d endTranslation = m_endPose.getTranslation();
    Translation2d desiredDifference = endTranslation.minus(currentTranslation);
    double desiredHeading = Math.atan2(desiredDifference.getY(), desiredDifference.getX());
    double heading = 360.0 - Units.radiansToDegrees(desiredHeading);
    double distanceRemaining = Units.metersToInches(endTranslation.getDistance(currentTranslation));

    SmartDashboard.putNumber("Profile Velocity", setPoint.velocity);
    SmartDashboard.putNumber("AUTO Target Heading", (heading >= 360.0) ? heading - 360.0 : heading);
    SmartDashboard.putNumber("AUTO Target Distance", distanceRemaining);


    ang = new Rotation2d(desiredHeading);
    if(distanceRemaining > Units.inchesToMeters(6) && distanceRemaining < Units.inchesToMeters(12)) {
      throttle = setPoint.velocity;
    } else if(distanceRemaining <= POSE_THRESHOLD_INCHES) {
      throttle = 0.0;
    }

    desiredMovement = new SwerveModuleState(throttle, ang /*new Rotation2d(0.0)*/);

    SmartDashboard.putNumber("AUTO Desired Throttle (m/s)", throttle);

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
