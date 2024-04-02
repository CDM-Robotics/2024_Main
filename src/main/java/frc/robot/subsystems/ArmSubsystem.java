package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.DrivingMotor;
import frc.robot.devices.SnowBlower;


public class ArmSubsystem extends SubsystemBase  {
    SnowBlower positionMotor;
    DrivingMotor intakeMotor;
    private EngineerController m_engineerController;
    private POSITION lastCommandedPosition;

    public ArmSubsystem(EngineerController engineerController) {
        positionMotor = new SnowBlower(17);
        m_engineerController = engineerController;
        lastCommandedPosition = POSITION.NONE;
    }

    public void initialize() {
        positionMotor.initialize();
    }

    public void setPosition(POSITION pos) {
        lastCommandedPosition = pos;
        if(pos == POSITION.AMP) {
            positionMotor.setAngle(Rotation2d.fromDegrees(Constants.SNOW_BLOWER_AMP_ANGLE));
        } else if(pos == POSITION.SOURCE) {
            positionMotor.setAngle(Rotation2d.fromDegrees(Constants.SNOW_BLOWER_SOURCE_ANGLE));
        }
    }

    public POSITION getPosition() {
        return lastCommandedPosition;
    }

    public boolean gotoStartPosition() {
        double currentAngle;
        double desiredMinAngle, desiredMaxAngle;

        currentAngle = positionMotor.getAngle();

        desiredMinAngle = Constants.SNOW_BLOWER_START_ANGLE - 10.0;
        desiredMaxAngle = Constants.SNOW_BLOWER_START_ANGLE + 10.0;
        if(RobotBase.isSimulation()) {
            currentAngle = (desiredMaxAngle + desiredMinAngle) / 2.0;
        }
        if(currentAngle < desiredMinAngle) {
            positionMotor.turnMotor(.2);
        } else if(currentAngle > desiredMaxAngle) {
            positionMotor.turnMotor(-.2);
        } else {
            positionMotor.turnMotor(0.0);
            return true;
        }

        return false;
    }

    @Override
    public void periodic() {
        POSITION desiredPosition;
        double desiredMinAngle, desiredMaxAngle;
        double currentAngle;

        if(!DriverStation.isAutonomous()) {
            desiredPosition = m_engineerController.getDesiredPosition();
        } else {
            desiredPosition = lastCommandedPosition;
        }
        currentAngle = positionMotor.getAngle();

        SmartDashboard.putNumber("SnowBlower Angle", currentAngle);

        switch (desiredPosition) {
            case AMP:
                desiredMinAngle = Constants.SNOW_BLOWER_AMP_ANGLE - 10.0;
                desiredMaxAngle = Constants.SNOW_BLOWER_AMP_ANGLE + 10.0;
                if(RobotBase.isSimulation()) {
                    currentAngle = (desiredMaxAngle + desiredMinAngle) / 2.0;
                }
                if(currentAngle < desiredMinAngle) {
                    positionMotor.turnMotor(.2);
                } else if(currentAngle > desiredMaxAngle) {
                    positionMotor.turnMotor(-.2);
                } else {
                    positionMotor.turnMotor(0.0);
                }
                break;

            case SOURCE:
                desiredMinAngle = Constants.SNOW_BLOWER_SOURCE_ANGLE - 10.0;
                desiredMaxAngle = Constants.SNOW_BLOWER_SOURCE_ANGLE + 10.0;
                if(RobotBase.isSimulation()) {
                    currentAngle = (desiredMaxAngle + desiredMinAngle) / 2.0;
                }
                if(currentAngle < desiredMinAngle) {
                    positionMotor.turnMotor(.2);
                } else if(currentAngle > desiredMaxAngle) {
                    positionMotor.turnMotor(-.2);
                } else {
                    positionMotor.turnMotor(0.0);
                }
                break;
        
            case START:
                gotoStartPosition();
                break;
            default:
                positionMotor.turnMotor(0.0);
                break;
            }
    }
}
