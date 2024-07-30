package frc.robot.testing;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DioTestRobot extends TimedRobot {
    private DigitalInput[] m_digitalInputs = {new DigitalInput(0), new DigitalInput(1), new DigitalInput(2), new DigitalInput(3), new DigitalInput(4), new DigitalInput(5), new DigitalInput(6), new DigitalInput(7), new DigitalInput(8), new DigitalInput(9)};

    @Override
    public void robotPeriodic() {
        for (int i = 0; i < m_digitalInputs.length; i++) {
            SmartDashboard.putBoolean("DIO " + i, m_digitalInputs[i].get());
        }
    }
}
