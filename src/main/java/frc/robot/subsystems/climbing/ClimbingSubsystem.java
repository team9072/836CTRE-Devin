package frc.robot.subsystems.climbing;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
    private final ClimbingPneumatics m_pneumatics = new ClimbingPneumatics();

    //TODO: Climbing motors 

    public void raiseArms() {
        m_pneumatics.raiseArms();
    }

    public void lowerArms() {
        m_pneumatics.lowerArms();
    }

    public boolean areArmsRaised() {
        return m_pneumatics.areArmsRaised();
    }
}
