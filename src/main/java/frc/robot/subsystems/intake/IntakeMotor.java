package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants.Ids;

class IntakeMotor {
    public enum IntakeMotorState {
        STOPPED(0),
        INTAKE(0.7);

        double speed;

        IntakeMotorState(double speed) {
            this.speed = speed;
        }
    }

    private final VictorSPX m_intakeMotor = new VictorSPX(Ids.kIntakeMotorCanId);
    private IntakeMotorState m_state = IntakeMotorState.STOPPED;

    public void set(IntakeMotorState state) {
        m_state = state;
        m_intakeMotor.set(VictorSPXControlMode.PercentOutput, m_state.speed);
    }

    public IntakeMotorState getState() {
        return m_state;
    }
}
