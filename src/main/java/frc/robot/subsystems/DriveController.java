package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveAlignToAngle;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveController extends SubsystemBase {
    private XboxController xbox;

    private double desiredAngle;
    private double m_vx;
    private double m_vy;
    private double desiredThrottle;
    private double rotation;
    private boolean m_rotateToSource;
    private DriveAlignToAngle m_driveAlignToAngle;
    private DriverStation.Alliance m_alliance;
    private boolean m_overrideAutoThrottle;
    private boolean m_reInit;

    public DriveController(DriveAlignToAngle driveAlignToAngle) {
        xbox = new XboxController(0);
        m_rotateToSource = false;
        m_overrideAutoThrottle = false;
        m_reInit = false;
        m_driveAlignToAngle = driveAlignToAngle;
        DriverStation.Alliance m_alliance = null;

        if(DriverStation.getAlliance().isPresent()) {
            m_alliance = DriverStation.getAlliance().get();
            SmartDashboard.putString("Alliance", m_alliance.name());
        } else {
            SmartDashboard.putString("Alliance", "WARNING, NOT SET!!!");
        }
    }

    @Override
    public void periodic() {
        double x, y, mag;
        double m_rampSourceAngle = 0.0;
        double m_armSourceAngle = 0.0;
        double m_speakerAngle = 0.0;
        double m_ampAngle = 0.0;
        
        if(m_alliance != null) {
            if(m_alliance == DriverStation.Alliance.Blue) {
                m_rampSourceAngle = Constants.BLUE_RAMP_SOURCE_ANGLE;
                m_armSourceAngle = Constants.BLUE_ARM_SOURCE_ANGLE;
                m_ampAngle = Constants.BLUE_AMP_ANGLE;
            } else {
                m_rampSourceAngle = Constants.RED_RAMP_SOURCE_ANGLE;
                m_armSourceAngle = Constants.RED_ARM_SOURCE_ANGLE;
                m_ampAngle = Constants.RED_AMP_ANGLE;
            }
            m_speakerAngle = Constants.SPEAKER_ANGLE;
        } else {
            if(DriverStation.getAlliance().isPresent()) {
                m_alliance = DriverStation.getAlliance().get();
                SmartDashboard.putString("Alliance", m_alliance.name());
            }
            /*DriverStation.get
            String astr = SmartDashboard.getString("Alliance", "WARNING, NOT SET!!!");
            if(astr.compareTo("Blue") == 0) {
                m_alliance = DriverStation.Alliance.Blue;
            } else if(astr.compareTo("Red") == 0) {
                m_alliance = DriverStation.Alliance.Red;
            }
            SmartDashboard.putData( );*/
        }

        m_overrideAutoThrottle = xbox.getLeftBumper();

        if(xbox.getXButton()) {
            m_driveAlignToAngle.setAngle(m_rampSourceAngle);
            m_driveAlignToAngle.schedule();
        } else if(xbox.getYButton()) {
            m_driveAlignToAngle.setAngle(m_armSourceAngle);
            m_driveAlignToAngle.schedule();
        } else if(xbox.getAButton()) {
            m_driveAlignToAngle.setAngle(m_ampAngle);
            m_driveAlignToAngle.schedule();
        } else if(xbox.getBButton()) {
            m_driveAlignToAngle.setAngle(m_speakerAngle);
            m_driveAlignToAngle.schedule();
        }

        if(xbox.getBackButton()) {
            this.m_reInit = true;
        } else {
            this.m_reInit = false;
        }

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

    public boolean getRotateToSource() {
        return m_rotateToSource;
    }

    public boolean overrideAutoThrottle() {
        return true;
    }

    public boolean wantToZero() {
        return this.m_reInit;
    }
    
}
