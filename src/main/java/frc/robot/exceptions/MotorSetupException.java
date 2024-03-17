package frc.robot.exceptions;

public class MotorSetupException extends Exception {
    public MotorSetupException(String message) {
        super(message);
    }

    public MotorSetupException(Throwable t) {
        super(t);
    }
}