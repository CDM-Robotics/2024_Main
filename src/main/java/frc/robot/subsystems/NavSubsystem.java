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
    static double localAngle;
    private int debugNavReconnectCnt;

    {
        rawAngle = 0.0;
        offsetAngle = 0.0;
        localAngle = 0.0;
    }

    public NavSubsystem(NavX navx) {
        this.navx = navx;
        this.navx.init();
        initialized = false;
        updating = 0;
        debugNavReconnectCnt = 0;
    }

    @Override
    public void periodic() {
        double angle;

        debugNavReconnectCnt++;
        if (DriverStation.isDisabled()) {
            angle = this.navx.getContinuousAngle();
            NavSubsystem.setRawAngle(angle);
            //Logger.getInstance().recordOutput("Raw Angle", getRawAngle());
        }

        if(debugNavReconnectCnt%50 == 0) {
            SmartDashboard.putNumber("NAV Reconnects", this.navx.getReconnectCount());
            debugNavReconnectCnt = debugNavReconnectCnt%50;
        }
    }

    public void update() {
        double angle;

        angle = this.navx.getContinuousAngle();
        NavSubsystem.setRawAngle(angle);

        angle = this.navx.getFieldAngle();
        localAngle = angle;

        if(!initialized) {
            offsetAngle = getRawAngle();
            initialized = true;
        }
    }

    public static synchronized void setRawAngle(double a) {
        rawAngle = a;
    }

    public static synchronized double getRawAngle() {
        return rawAngle;
    }

    public static synchronized double getContinuousAngle() {
        return rawAngle - offsetAngle;
    }

    public static synchronized void setLocalAngle(double a) {
        localAngle = a;
    }

    public static synchronized double getLocalAngle() {
        return localAngle - offsetAngle;
    }

    public static Rotation2d getRotation() {
        return new Rotation2d(getContinuousAngle());
    }

    public void reInitialize() {
        this.initialized = false;
    }
}
