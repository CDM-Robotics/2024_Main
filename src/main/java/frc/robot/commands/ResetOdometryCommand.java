package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ResetOdometryCommand extends Command {
    DriveSubsystem  m_driveSubsystem;
    Trajectory m_trajectory;
    static int count;

    {count = 0;}

    public ResetOdometryCommand(DriveSubsystem p_drivetrain, Trajectory p_trajectory) {
        m_driveSubsystem = p_drivetrain;
        m_trajectory = p_trajectory;
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    count++;
    SmartDashboard.putNumber("ResetOdometryCount", count);
    
    return true;
  }
}
