package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class DriveController extends SubsystemBase {
    private XboxController xbox;

    private double desiredAngle;
    private double m_vx;
    private double m_vy;
    private double desiredThrottle;
    private double rotation;

    public DriveController() {
        xbox = new XboxController(0);
    }

    @Override
    public void periodic() {
        double x, y, mag;

        y = -xbox.getLeftX();  // +/- 1.0
        x = -xbox.getLeftY(); // +/- 1.0  Forward is negative, go figure  
        rotation = -xbox.getRightX();  // Left is positive

        if(Math.abs(rotation) < 0.1) { 
            rotation = 0.0;
        }


        // At 45 degrees, both x and y can have values of 1.0, so we need to normalize
        // the values into unit vectors
        mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        if(mag > 1.0) {
            x = x / mag;
            y = y / mag;
            mag = 1.0;
        }

        // This isn't necessary, since the sqrt will always be positive
        desiredThrottle = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));;
        
        desiredThrottle = MathUtil.applyDeadband(desiredThrottle, 0.25,1.0);
        /*if(Math.abs(desiredThrottle) < 0.25) {
            desiredThrottle = 0.0;
        }*/
        
        if(desiredThrottle > 1.0) {
            desiredThrottle = 1.0;
        } else if(desiredThrottle < -1.0) {
            desiredThrottle = -1.0;
        }

        desiredThrottle = desiredThrottle * Constants.MAX_VELOCITY;

        // Math.acos only returns positive values from 0 to pi
        desiredAngle = Math.acos(y) / Math.PI * 180.0;  // The hypotenuse = 1 for unit circle
        if(x < 0.0) {
            desiredAngle = -desiredAngle;
        }

        m_vx = x;
        m_vy = y;

        // Apparently, CCW is a positive rotation
        desiredAngle = -desiredAngle;
    }

    public double getDesiredThrottle() {
        return desiredThrottle;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getDesiredVX() {
        return m_vx;
    }

    public double getDesiredVY() {
        return m_vy;
    }

    public boolean allowedToRotate() {
        return (desiredThrottle == 0.0);
    }

    public double getRotation() {
        return rotation;
    }
    
}
