package frc.robot.devices;

import edu.wpi.first.hal.SerialPortJNI;


public class UltrasonicRangefinder implements Runnable {

    private String m_serialPort;
    private int m_handle;
    private boolean m_initialized;
    private int m_id;
    private int timeouts;
    private int measurements;

    private double m_range;

    public UltrasonicRangefinder(int id, String port) {
        m_id = id;
        m_serialPort = port;
        m_initialized = false;
        timeouts = 0;
        measurements = 0;
    }

    public boolean init() {
        m_handle = SerialPortJNI.serialInitializePortDirect((byte)m_id, m_serialPort);

        if(m_handle > 0) {
            SerialPortJNI.serialSetBaudRate(m_handle, 9600);
            SerialPortJNI.serialSetDataBits(m_handle, (byte)8);
            SerialPortJNI.serialSetParity(m_handle, (byte)0);
            SerialPortJNI.serialSetStopBits(m_handle,(byte)10);
            SerialPortJNI.serialSetFlowControl(m_handle, (byte)0);
            SerialPortJNI.serialSetTimeout(m_handle, 120);  // Samples at 10Hz + 20% margin
            SerialPortJNI.serialEnableTermination(m_handle, '\r');
            m_initialized = true;
        } else {
            System.out.println("Could not open serial port (" + m_serialPort + ")");
        }
        
        return m_initialized;
    }

    public void run() {
        byte data[] = new byte[1000];

        if(m_handle > 0) {
            SerialPortJNI.serialClear(m_handle);
            while(!Thread.interrupted()) {
                if(SerialPortJNI.serialRead(m_handle, data, 6) > 0) {
                    try {
                        String ascii = new String(data, "US-ASCII").trim();
                        
                        if(ascii.length() == 5) {
                            if(ascii.substring(0, 1).compareTo("R") == 0) {
                                String distanceAsString = ascii.substring(1,5);
                                if(measurements >= 3) {
                                    setRange(Double.parseDouble(distanceAsString));
                                }
                                measurements++;
                                timeouts = 0;
                            }
                        } 
                    } catch (Exception e) {}
                } else {
                    if(timeouts >= 3) {
                        measurements = 0;
                        setRange(-999999.0);
                    }
                    timeouts++;
                }
            }

            SerialPortJNI.serialClose(m_handle);
        }
    }

    public synchronized void setRange(double r) {
        m_range = r;
    }

    public synchronized double getRange() {
        if(m_initialized) {
            return m_range;
        } else {
            return 0.0;
        }
    }
}