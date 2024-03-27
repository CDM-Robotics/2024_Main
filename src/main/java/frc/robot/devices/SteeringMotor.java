package frc.robot.devices;

import frc.robot.Constants;
import frc.robot.exceptions.MotorSetupException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SteeringMotor extends CANSparkMax {
    private final double UNITS_PER_REV = 2048.0;
    private final AbsoluteEncoder m_turningEncoder;
    private final SparkPIDController m_turningPIDController;
    private Rotation2d m_desiredAngle;

    public SteeringMotor(int canID, boolean simulate) {
        super(canID, MotorType.kBrushless);

        // Can only be assigned once in the constructor
        m_turningEncoder = getAbsoluteEncoder(Type.kDutyCycle);
        m_turningPIDController = getPIDController();
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    }

    public void initialize() throws MotorSetupException {
        double posFactor, desiredPosFactor;
        double velFactor, desiredVelFactor;
        boolean inverted, desiredInverted;
        boolean posPIDWrapEnabled, desiredPosPIDWrapEnabled;
        double posPIDWrapMin, desiredPosPIDWrapMin;
        double posPIDWrapMax, desiredPosPIDWrapMax;
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
        posFactor = m_turningEncoder.getPositionConversionFactor();
        velFactor = m_turningEncoder.getVelocityConversionFactor();
        inverted = m_turningEncoder.getInverted();
        posPIDWrapEnabled = m_turningPIDController.getPositionPIDWrappingEnabled();
        posPIDWrapMin = m_turningPIDController.getPositionPIDWrappingMinInput();
        posPIDWrapMax = m_turningPIDController.getPositionPIDWrappingMaxInput();
        p = m_turningPIDController.getP();
        i = m_turningPIDController.getI();
        d = m_turningPIDController.getD();
        ff = m_turningPIDController.getFF();
        outRangeMin = m_turningPIDController.getOutputMin();
        outRangeMax = m_turningPIDController.getOutputMax();
        idle = getIdleMode();

        // Desired values
        resetAll = false;
        desiredPosFactor = 2 * Math.PI;
        desiredVelFactor = ((2 * Math.PI) / 60);
        desiredInverted = true;
        desiredPosPIDWrapEnabled = true;
        desiredPosPIDWrapMin = 0.0;
        desiredPosPIDWrapMax = 2 * Math.PI;
        desiredP = 1.0;
        desiredI = 0.0;
        desiredD = 0.0;
        desiredFF = 0.0;
        desiredOutRangeMin = -1.0;
        desiredOutRangeMax = 1.0;
        desiredIdle = IdleMode.kBrake;

        // All doubles shall be +/- 5% with the same sign!!!
        // All integers shall be exact
        if((posFactor < (desiredPosFactor * 0.95)) || (posFactor > (desiredPosFactor * 1.05))) {
            resetAll = true;
        }
        if((velFactor < (desiredVelFactor * 0.95)) || (velFactor > (desiredVelFactor * 1.05))) {
            resetAll = true;
        }
        if(inverted != desiredInverted) {
            resetAll = true;
        }
        if(posPIDWrapEnabled != desiredPosPIDWrapEnabled) {
            resetAll = true;
        }
        if((posPIDWrapMin < (desiredPosPIDWrapMin * 0.95)) || (posPIDWrapMin > (desiredPosPIDWrapMin * 1.05))) {
            resetAll = true;
        }
        if((posPIDWrapMax < (desiredPosPIDWrapMax * 0.95)) || (posPIDWrapMax > (desiredPosPIDWrapMax * 1.05))) {
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

        // Be as defensive as possible.  The Spark(s) seem to be having problems initializing sometimes
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
            while(/*true*/((error = m_turningEncoder.setPositionConversionFactor(desiredPosFactor)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningEncoder.setVelocityConversionFactor(desiredVelFactor)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningEncoder.setInverted(desiredInverted)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setPositionPIDWrappingEnabled(desiredPosPIDWrapEnabled)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setPositionPIDWrappingMinInput(desiredPosPIDWrapMin)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setPositionPIDWrappingMaxInput(desiredPosPIDWrapMax)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setP(desiredP)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setI(desiredI)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setD(desiredD)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setFF(desiredFF)) != REVLibError.kOk) && (attempts<= 3)) {
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
            while(((error = m_turningPIDController.setOutputRange(desiredOutRangeMin, desiredOutRangeMax)) != REVLibError.kOk) && (attempts<= 3)) {
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
        setSmartCurrentLimit(Constants.steeringMotorCurrentLimit);

        if(resetAll) {
            burnFlash();
        }

        m_desiredAngle = new Rotation2d(m_turningEncoder.getPosition());
    }

    public Rotation2d getAngle() {
        return new Rotation2d(m_turningEncoder.getPosition());
    }

    public void setAngle(Rotation2d angle) {
        m_turningPIDController.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
        m_desiredAngle = angle;
    }

    public void setAngle(double angle) {
        setAngle(new Rotation2d(angle));
    }

    public double getPosition() {
        return m_turningEncoder.getPosition();
    }
}
