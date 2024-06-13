package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ids;

public class Flywheel extends SubsystemBase {
    private static final double flywheelKF = 1023 / 44205.9;

    private final TalonSRX m_shooterMotor = new TalonSRX(Ids.kFlywheelMasterMotorCanId);

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

    public void stop() {
        m_shooterMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    /**
     * Set the flywheel speed in raw sensor units
     * @param velocity
     */
    public void setVelocityRaw(double velocity) {
        m_shooterMotor.set(TalonSRXControlMode.Velocity, velocity);
    }

    /**
     * Get the current flywheel velocity 
     * @return flywheel velocity (in raw sensor units) per 100ms
     */
    public double getVelocity() {
        return m_shooterMotor.getSelectedSensorVelocity();
    }

    /**
     * Get the error between the flywheel's expected and actual velocity
     * @return error in flywheel velocity (in raw sensor units) per 100ms
     */
    public double getVelocityErrorRaw() {
        return m_shooterMotor.getClosedLoopError();
    }

    /**
     * Is the flywheel velocity error less than a supplied maximum
     * @param maxVelocityError minimum velocity error (in raw sensor units) of the flywheel
     * @return true if flywheel is up to speed
     */
    public boolean isAtTargetVelocity(double maxVelocityError) {
        return Math.abs(getVelocityErrorRaw()) < maxVelocityError;
    }

    /**
     * Is the flywheel velocity error small enough
     * @return true if flywheel is up to speed
     */
    public boolean isAtTargetVelocity() {
        return isAtTargetVelocity(2000);
    }
}
