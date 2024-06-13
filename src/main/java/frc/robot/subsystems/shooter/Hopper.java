package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ids;

public class Hopper extends SubsystemBase {
    public enum HopperMotorState {
        STOPPED(0),
        RUNNING(0.5),
        SHOOTING(1);

        double speed;

        HopperMotorState(double speed) {
            this.speed = speed;
        }
    }

    private static final double kSensorDebounceTime = 0.1;
    
    private final VictorSPX m_motor = new VictorSPX(Ids.kHopperMotorCanId);
    private final DigitalInput m_primeSensor = new DigitalInput(Ids.kPrimeSensorDioId);
    private final DigitalInput m_innerIntakeSensor = new DigitalInput(Ids.kInnerBallSensorDioId);
    private final DigitalInput m_outerIntakeSensor = new DigitalInput(Ids.kOuterBallSensorDioId);
    // Sensors go low when detecting a ball
    private final Trigger m_primeTrigger = new Trigger(() -> !m_primeSensor.get()).debounce(kSensorDebounceTime);
    // Trigger when either detects a ball, (when both are not high)
    private final Trigger m_intakeBallTrigger = new Trigger(() -> !(m_innerIntakeSensor.get() && m_outerIntakeSensor.get())).debounce(kSensorDebounceTime);
    
    private HopperMotorState m_state = HopperMotorState.STOPPED;

    public void set(HopperMotorState state) {
        m_state = state;
        m_motor.set(VictorSPXControlMode.PercentOutput, m_state.speed);
    }
    
    public Trigger getPrimeTrigger() {
        return this.m_primeTrigger;
    }

    public Trigger getIntakeBallTrigger() {
        return this.m_intakeBallTrigger;
    }

    public boolean isPrimed() {
        return m_primeTrigger.getAsBoolean();
    }
}
