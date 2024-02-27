package frc.robot.devices;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

public class NavX {
    private AHRS ahrs;
    private double offset;

    public NavX() {
    }

    public void init() {
        try {
            ahrs = new AHRS(Port.kUSB);
            ahrs.zeroYaw();
            offset = ahrs.getYaw();
        } catch(Exception e) {
            System.out.println("Error instantiating NavX");
        }
    }

    public double getFieldAngle() {
        double angle = ahrs.getYaw() + 180.0;
        if(angle > 360.0) {
            angle = 360.0;
        }

        return angle;
    }

    public double getContinuousAngle() {
        return ahrs.getAngle();
    }
}
