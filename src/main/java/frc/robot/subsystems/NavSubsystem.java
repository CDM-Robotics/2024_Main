package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.devices.NavX;

public class NavSubsystem extends SubsystemBase {

    private NavX navx;
    static double rawAngle;
    private boolean initialized;
    static double offsetAngle;
    private int updating;

    {
        rawAngle = 0.0;
        offsetAngle = 0.0;
    }

    public NavSubsystem(NavX navx) {
        this.navx = navx;
        this.navx.init();
        initialized = false;
        updating = 0;
    }

    @Override
    public void periodic() {
        double angle;
        if (DriverStation.isDisabled()) {
            angle = this.navx.getFieldAngle();
            NavSubsystem.setRawAngle(angle);
            //Logger.getInstance().recordOutput("Raw Angle", getRawAngle());
        }
    }

    public void update() {
        double angle;

        angle = this.navx.getFieldAngle();
        NavSubsystem.setRawAngle(angle);

        SmartDashboard.putNumber("Raw Angle", angle);
        if(!initialized) {
            offsetAngle = getRawAngle();
            initialized = true;
            SmartDashboard.putNumber("Start Angle", offsetAngle);
        }

    }

    public static synchronized void setRawAngle(double a) {
        rawAngle = a;
    }

    public static synchronized double getRawAngle() {
        return rawAngle;
    }

    public static synchronized double getFieldAngle() {
        return rawAngle - offsetAngle;
    }

    public static Rotation2d getRotation() {
        return new Rotation2d(getFieldAngle());
    }
}
