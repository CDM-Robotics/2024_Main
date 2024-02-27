package frc.robot.devices;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SnowBlower extends CANSparkMax {
    private final AbsoluteEncoder turningEncoder;
    //private final SparkPIDController turningPIDController;
    private Rotation2d desiredAngle;

    public SnowBlower(int canID) {
        super(canID,MotorType.kBrushed);

        // Can only be assigned once in the constructor
        turningEncoder = getAbsoluteEncoder(Type.kDutyCycle);
        //turningPIDController = getPIDController();
        //turningPIDController.setFeedbackDevice(turningEncoder);
    }

    public void initialize() {
        turningEncoder.setPositionConversionFactor(360.0);
        /*turningPIDController.setP(0.04);
        turningPIDController.setI(0.0);
        turningPIDController.setD(0.0);
        turningPIDController.setFF(1 / Constants.kDriveWheelFreeSpeedRps);
        turningPIDController.setOutputRange(-1, 1);*/
        setIdleMode(IdleMode.kBrake);
        setSmartCurrentLimit(15);
        //setSoftLimit(SoftLimitDirection.kForward, 95.0f / 180.0f * (float)Math.PI);
        //setSoftLimit(SoftLimitDirection.kReverse, -5.0f / 180.0f * (float)Math.PI);
        enableSoftLimit(SoftLimitDirection.kForward, false);
        enableSoftLimit(SoftLimitDirection.kReverse,false);
        
        // The encoder is mounted reversed, so we need to set the motor control to inverted
        // DO NOT CHANGE THIS!!!!!1
        setInverted(true);

        burnFlash();

        
    }

    public void turnMotor(double percent) {
        setVoltage(percent * 12.5);  // Nominal voltage out
    }

    public void setAngle(Rotation2d angle) {
        SmartDashboard.putNumber("SnowBlower Angle", turningEncoder.getPosition());
        //desiredAngle = angle;
    }
    
    public double getAngle() {
        return turningEncoder.getPosition();
    }
}
