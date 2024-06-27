package frc.robot.subsystems.climbing;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.Ids;

class ClimbingPneumatics {
    private final DoubleSolenoid m_armsSolenoid = new DoubleSolenoid(Ids.kPneumaticControllerCanId, PneumaticsModuleType.CTREPCM, Ids.kClimberArmsSolenoidChannel, 4);

    public void raiseArms() {
        m_armsSolenoid.set(Value.kForward);
    }

    public void lowerArms() {
        m_armsSolenoid.set(Value.kReverse);
    }

    public boolean areArmsRaised() {
        return m_armsSolenoid.get() == Value.kForward;
    }
}
