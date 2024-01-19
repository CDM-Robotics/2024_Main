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
        return ahrs.getAngle();
    }
}
