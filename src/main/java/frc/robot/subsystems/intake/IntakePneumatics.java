package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.Ids;

public class IntakePneumatics {
    private final Solenoid m_extenderSolenoid = new Solenoid(Ids.kPneumaticControllerCanId, PneumaticsModuleType.CTREPCM, Ids.kIntakeSolenoidChannel);

    public void deploy() {
        m_extenderSolenoid.set(true);
    }

    public void retract() {
        m_extenderSolenoid.set(false);
    }

    public boolean isDeployed() {
        return m_extenderSolenoid.get();
    }
}
