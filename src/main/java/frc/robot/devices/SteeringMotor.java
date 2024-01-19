package frc.robot.devices;

import frc.robot.exceptions.MotorSetupException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SteeringMotor extends CANSparkMax {
    private final double UNITS_PER_REV = 2048.0;
    private double simValue;
    private int m_numWraps;
    private double m_currentSensorPos;
    private double m_currentAngle;
    private double m_memorizedCommandAngle;
    private boolean m_sim;
    private int myID;
    private final AbsoluteEncoder m_turningEncoder;
    private final SparkMaxPIDController m_turningPIDController;
    private Rotation2d m_desiredAngle;

    public SteeringMotor(int canID, boolean simulate) {
        super(canID, MotorType.kBrushless);
        simValue = 0.0;
        m_sim = simulate;
        myID = canID;

        // Can only be assigned once in the constructor
        m_turningEncoder = getAbsoluteEncoder(Type.kDutyCycle);
        m_turningPIDController = getPIDController();
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    }

    public void initialize() throws MotorSetupException {
        restoreFactoryDefaults();

        m_turningEncoder.setPositionConversionFactor(2 * Math.PI);
        m_turningEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);
        m_turningEncoder.setInverted(true);
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(0);
        m_turningPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);  // Same as Position Conversion Factor
        m_turningPIDController.setP(1);
        m_turningPIDController.setI(0);
        m_turningPIDController.setD(0);
        m_turningPIDController.setFF(0);
        m_turningPIDController.setOutputRange(-1, 1);

        setIdleMode(IdleMode.kBrake);
        setSmartCurrentLimit(20);
        burnFlash();

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
}
