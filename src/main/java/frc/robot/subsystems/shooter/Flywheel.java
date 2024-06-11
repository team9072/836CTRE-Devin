package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.Ids;

public class Flywheel {
    private TalonSRX m_shooterMotor = new TalonSRX(Ids.kFlywheelMasterMotorCanId);
    private static final double flywheelKF = 1023 / 44205.9;

    public Flywheel() {
        m_shooterMotor.configFactoryDefault();
        m_shooterMotor.configVoltageCompSaturation(12);
        m_shooterMotor.enableVoltageCompensation(true);
        m_shooterMotor.setSensorPhase(true);
        m_shooterMotor.configPeakOutputForward(1);
        m_shooterMotor.configPeakOutputReverse(0);
        m_shooterMotor.config_kP(0, 0);
        m_shooterMotor.config_kI(0, 0);
        m_shooterMotor.config_kD(0, 0);
        m_shooterMotor.config_kF(0, flywheelKF);
        m_shooterMotor.setNeutralMode(NeutralMode.Coast);

        VictorSPX follower = new VictorSPX(Ids.kFlywheelFollowerMotorCanId);
        follower.configFactoryDefault();
        follower.setInverted(InvertType.OpposeMaster);
        follower.follow(m_shooterMotor);
    }

    public void set(double speed) {
        m_shooterMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setVelocity(double velocity) {
        m_shooterMotor.set(TalonSRXControlMode.Velocity, velocity);
    }

    /**
     * Get the current flywheel velocity 
     * @return flywheel velocity (in raw sensor units) per 100ms
     */
    public double getVelocity() {
        return m_shooterMotor.getSelectedSensorVelocity();
    }
}
