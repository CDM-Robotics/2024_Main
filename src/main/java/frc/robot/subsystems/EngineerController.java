package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class EngineerController extends SubsystemBase {
    private XboxController xbox;
    private POSITION pos;
    private boolean speakerFireAway;
    private boolean speakerSourceIntake;

    public EngineerController() {
        xbox = new XboxController(1);
        pos = POSITION.NONE;
    }

    @Override
    public void periodic() {
        if(xbox.getYButtonPressed()) {
            pos = POSITION.SOURCE;
        } else if(xbox.getAButtonPressed()) {
            pos = POSITION.AMP;
        } 

        // Check buttons for tasks to execute
        speakerFireAway = xbox.getXButton();
        speakerSourceIntake = xbox.getBButton();
    }

    public POSITION getDesiredPosition() {
        return pos;
    }

    public boolean speakerReadyToFire() {
        return speakerFireAway;
    }

    public boolean speakerReadyToIntake() {
        return speakerSourceIntake;
    }
}
