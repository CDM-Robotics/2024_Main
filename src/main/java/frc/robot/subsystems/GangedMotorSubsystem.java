// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.devices.VelocityMotor;
import frc.robot.exceptions.MotorSetupException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.CANSparkBase.IdleMode;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class GangedMotorSubsystem extends SubsystemBase {
    private VelocityMotor v1;
    private VelocityMotor v2;
    private int counter = 0;
    
    public GangedMotorSubsystem(int canID1, int canID2) {
        v1 = new VelocityMotor(canID1);
        v2 = new VelocityMotor(canID2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Log empty setpoint states when disabled
        /*if (DriverStation.isDisabled()) {
            Logger.getInstance().recordOutput("Odometry", new SwerveModuleState[] {});
        }*/
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public Boolean initialize() {
        try {
            v1.initialize();
            v2.initialize();
            v1.setIdleMode(IdleMode.kCoast);
            v2.setIdleMode(IdleMode.kCoast);
            v1.setVelocity(0.0);
            v2.setVelocity(0.0);
        } catch(Exception e) {
            return false;
        }

        return true;
    }

    public void setVelocity(double s) {
        if(s == 0.0) {
            v1.stopMotor();
            v2.stopMotor();
            counter = 0;
            return;
        }
        
        v2.setVelocity(s);
        counter++;
        if ((s < 0.0) || (counter >= 50))
            v1.setVelocity(s);
        if (s == 0) 
            counter = 0;
        
    }
}
