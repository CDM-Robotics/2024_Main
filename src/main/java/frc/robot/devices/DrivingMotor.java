// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.devices;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.exceptions.MotorSetupException;
import frc.robot.Constants;

/** Add your docs here. */
//public class DrivingMotor extends CANSparkMax {
public class DrivingMotor extends CANSparkFlex {
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
        double posFactor, desiredPosFactor;
        double velFactor, desiredVelFactor;
        double p, desiredP;
        double i, desiredI;
        double d, desiredD;
        double ff, desiredFF;
        double outRangeMin, outRangeMax, desiredOutRangeMin, desiredOutRangeMax;
        IdleMode idle, desiredIdle;
        boolean resetAll;
        int attempts;
        REVLibError error;

        // Get the stored values to see if we need to reset
        posFactor = m_drivingEncoder.getPositionConversionFactor();
        velFactor = m_drivingEncoder.getVelocityConversionFactor();
        p = m_drivingPIDController.getP();
        i = m_drivingPIDController.getI();
        d = m_drivingPIDController.getD();
        ff = m_drivingPIDController.getFF();
        outRangeMin = m_drivingPIDController.getOutputMin();
        outRangeMax = m_drivingPIDController.getOutputMax();
        idle = getIdleMode();

        // Desired values
        resetAll = false;
        desiredPosFactor = (Constants.kWheelDiameterMeters * Math.PI) / Constants.kDrivingMotorReduction;
        desiredVelFactor = ((Constants.kWheelDiameterMeters * Math.PI) / Constants.kDrivingMotorReduction) / 60.0;
        desiredP = 0.04;
        desiredI = 0.0;
        desiredD = 0.0;
        desiredFF = 1 / Constants.kDriveWheelFreeSpeedRps;
        desiredOutRangeMin = -1;
        desiredOutRangeMax = 1;
        desiredIdle = IdleMode.kBrake;

        // All doubles shall be +/- 5%
        // All integers shall be exact
        if((posFactor < (desiredPosFactor * 0.95)) || (posFactor > (desiredPosFactor * 1.05))) {
            resetAll = true;
        }
        if((velFactor < (desiredVelFactor * 0.95)) || (velFactor > (desiredVelFactor * 1.05))) {
            resetAll = true;
        }
        if((p < (desiredP * 0.95)) || (p > (desiredP * 1.05))) {
            resetAll = true;
        }
        if((i < (desiredI * 0.95)) || (i > (desiredI * 1.05))) {
            resetAll = true;
        }
        if((d < (desiredD * 0.95)) || (d > (desiredD * 1.05))) {
            resetAll = true;
        }
        if((ff < (desiredFF * 0.95)) || (ff > (desiredFF * 1.05))) {
            resetAll = true;
        }
        if((outRangeMin != desiredOutRangeMin) || (outRangeMax != desiredOutRangeMax)) {
            resetAll = true;
        }
        if(idle != desiredIdle) {
            resetAll = true;
        }

        // Be as defensive as possible.  The SparkFex(s) seem to be having problems initializing sometimes
        if(resetAll) {
            restoreFactoryDefaults();
            try {
                Thread.sleep(100);
            } catch(Exception e) {
                throw new MotorSetupException(e);
            }

            attempts = 0;
            // For Test ONLY --- 
            // error = REVLibError.kTimeout;
            while(/*true*/((error = m_drivingEncoder.setPositionConversionFactor(desiredPosFactor)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setPositionConversionFactor (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = m_drivingEncoder.setVelocityConversionFactor(desiredVelFactor)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setVelocityConversionFactor (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = m_drivingPIDController.setP(desiredP)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setP (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = m_drivingPIDController.setI(desiredI)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setI (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = m_drivingPIDController.setD(desiredD)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setD (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = m_drivingPIDController.setFF(desiredFF)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setFF (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = m_drivingPIDController.setOutputRange(desiredOutRangeMin, desiredOutRangeMax)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setOutputRange (CAN ID: " + getDeviceId() + ")");
            }

            attempts = 0;
            while(((error = setIdleMode(IdleMode.kBrake)) != REVLibError.kOk) && (attempts<= 3)) {
                attempts++;
                System.out.println("#### REVLibError (CAN ID: " + getDeviceId() + ") ==> " + error.name() + ", attempts = " + attempts);
                try {
                    Thread.sleep(100);
                } catch(Exception e) {
                    System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                    throw new MotorSetupException(e);
                }
            }
            if(attempts >= 4) {
                System.out.println("#### REBOOT REBOOT REBOOT #### " + error.name());
                throw new MotorSetupException("Failed setIdleMode (CAN ID: " + getDeviceId() + ")");
            }
        }

        // No getter, so always set it explicitly!!!
        setSmartCurrentLimit(Constants.drivingMotorCurrentLimit);
        m_drivingEncoder.setPosition(0);

        if(resetAll) {
            burnFlash();
        }
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
