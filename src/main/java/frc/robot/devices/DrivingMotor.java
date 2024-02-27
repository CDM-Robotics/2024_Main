// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.devices;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.exceptions.MotorSetupException;
import frc.robot.Constants;

/** Add your docs here. */
public class DrivingMotor extends CANSparkMax {
    private final RelativeEncoder m_drivingEncoder;
    private final SparkPIDController m_drivingPIDController;

    public DrivingMotor(int canID) {
        super(canID, MotorType.kBrushless);

        // Can only be assigned once in the constructor
        m_drivingEncoder = getEncoder();
        m_drivingPIDController = getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    }

    public void initialize() throws MotorSetupException {
        restoreFactoryDefaults();

        m_drivingEncoder.setPositionConversionFactor((Constants.kWheelDiameterMeters * Math.PI) / Constants.kDrivingMotorReduction); // meters);
        m_drivingEncoder.setVelocityConversionFactor(((Constants.kWheelDiameterMeters * Math.PI) / Constants.kDrivingMotorReduction) / 60.0); // meters per second
        m_drivingPIDController.setP(0.04);
        m_drivingPIDController.setI(0.0);
        m_drivingPIDController.setD(0.0);
        m_drivingPIDController.setFF(1 / Constants.kDriveWheelFreeSpeedRps);
        m_drivingPIDController.setOutputRange(-1, 1);
        
        setIdleMode(IdleMode.kBrake);
        setSmartCurrentLimit(Constants.drivingMotorCurrentLimit);
        burnFlash();
        
        m_drivingEncoder.setPosition(0);
    }

    public void setVelocity(double metersPerSec) {
        m_drivingPIDController.setReference(metersPerSec, CANSparkMax.ControlType.kVelocity);
    }

    public double getVelocity() {
        return m_drivingEncoder.getVelocity();
    }

    public double getPosition() {
        return m_drivingEncoder.getPosition();
    }

}
