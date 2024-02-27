package frc.robot.devices;

import frc.robot.Constants;
import frc.robot.exceptions.MotorSetupException;

import com.revrobotics.CANSparkMax;
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
        setSmartCurrentLimit(Constants.steeringMotorCurrentLimit);
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

    public double getPosition() {
        return m_turningEncoder.getPosition();
    }
}
