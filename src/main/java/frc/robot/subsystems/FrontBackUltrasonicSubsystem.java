package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.UltrasonicRangefinder;

import frc.robot.Constants;

public class FrontBackUltrasonicSubsystem extends SubsystemBase {
    private UltrasonicRangefinder m_forward;
    private UltrasonicRangefinder m_reverse;
    private Thread thread1;
    private Thread thread2;
    private int m_baseId;

    int update;
    
    public FrontBackUltrasonicSubsystem(int baseid, String port1, String port2) {
        m_baseId = baseid;
        m_forward = new UltrasonicRangefinder(m_baseId, port1);
        m_reverse = new UltrasonicRangefinder(m_baseId + 1, port2);

        update=0;
    }

    public boolean init() {
        boolean t1 = false, t2 = false;

        try {
            t1 = m_forward.init();

            if(t1) {
                thread1 = new Thread(m_forward);
                thread1.start();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        
        try {
            t2 = m_reverse.init();

            if(t2) {
                thread2 = new Thread(m_reverse);
                thread2.start();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return t1 & t2;
    }

    @Override
    public void periodic() {
        double a = m_forward.getRange();
        double b = m_reverse.getRange();

        /*if(a < 450.0) {
            System.out.print("Forward: " + a);
        }

        if((a < 450.0) & (b < 450.0)) {
            System.out.print(", ");
        }

        if(b < 450.0) {
            System.out.print("Reverse: " + b);
        }

        if((a < 450.0) | (b < 450.0)) {
            System.out.println("");
        }*/
    }

    @Override
    public void simulationPeriodic() {
        if(update == 0) {
            System.out.println("Simultaion, no range available");
        }

        update++;
    }

    public double getLastForwardRange() {
        return m_forward.getRange() + Constants.FORWARD_ULTRASONIC_SENSOR_OFFSET;
    }

    public double getLastReverseRange() {
        return m_reverse.getRange();
    }
}
