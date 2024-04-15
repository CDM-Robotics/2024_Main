package frc.robot.devices;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

public class NavX {
    private AHRS ahrs;
    private double offset;
    private double lastGoodYaw;
    private double lastGoodAngle;
    private boolean good;
    private int badConnectionCnt;

    public NavX() {
        good = false;
        badConnectionCnt = 0;
    }

    public void init() {
        try {
            ahrs = new AHRS(Port.kUSB);
            ahrs.zeroYaw();
            offset = ahrs.getYaw();
            lastGoodYaw = offset;
            lastGoodAngle = ahrs.getAngle();
            good = true;
        } catch(Exception e) {
            System.out.println("Error instantiating NavX");
        }
    }

    public double getFieldAngle() {
        if(ahrs.isConnected()) {
            good = true;
            lastGoodYaw = ahrs.getYaw();
            double angle = lastGoodYaw + 180.0;
            if(angle > 360.0) {
                angle = 360.0;
            }
        } else {
            if(good) {
                badConnectionCnt++;
                good = false;
            }
        }

        return lastGoodYaw;
    }

    public double getContinuousAngle() {
        if(ahrs.isConnected()) {
            good = true;
            lastGoodAngle = ahrs.getAngle();
        } else {
            if(good) {
                badConnectionCnt++;
                good = false;
            }
        }

        return lastGoodAngle;
    }

    public int getReconnectCount() {
        return badConnectionCnt;
    }
}
