package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ids;

public class TurretSubsystem extends SubsystemBase {
    private static final double kSoftLimitThreshhold = 4487;
    private static final double kMaxToForwardOffset = 7140;

    private final TalonSRX m_turretMotor = new TalonSRX(Ids.kTurretMotorCanID);

    private double m_minEncoderValue = Double.NaN;
    private double m_maxEncoderValue = Double.NaN;

    public TurretSubsystem() {
        m_turretMotor.configFactoryDefault();
        m_turretMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        m_turretMotor.setSensorPhase(true);
        m_turretMotor.config_kP(0, 0.11);
        m_turretMotor.configVoltageCompSaturation(12);
        SmartDashboard.putNumber("pos", 0);
    }

    public void setOutput(double percent) {
        if (!isTurretCalibrated()) {
            disableTurret();
            return;
        }

        m_turretMotor.set(TalonSRXControlMode.PercentOutput, percent);
    }

    /**
     * Get the adjusted position of the turret
     * If turret is not calibrated, returns NaN
     */
    public double getPosition() {
        if (!isTurretCalibrated()) return Double.NaN;
        return m_turretMotor.getSelectedSensorPosition() + (kMaxToForwardOffset - m_maxEncoderValue);
    }

    public void setPosition(double position) {
        if (!isTurretCalibrated()) {
            disableTurret();
            return;
        }

        double minLimit = m_minEncoderValue + kSoftLimitThreshhold;
        double maxLimit = m_maxEncoderValue - kSoftLimitThreshhold;
        m_turretMotor.set(TalonSRXControlMode.Position, Math.max(minLimit, Math.min(position - (kMaxToForwardOffset - m_maxEncoderValue), maxLimit)));
    }

    private void disableTurret() {
        m_turretMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    private Command testTurretLimit(boolean reverse) {
        return Commands.sequence(
            this.runOnce(() -> {
                if (reverse) {
                    m_turretMotor.configReverseSoftLimitEnable(false);
                    m_minEncoderValue = Double.NaN;
                } else {
                    m_turretMotor.configForwardSoftLimitEnable(false);
                    m_maxEncoderValue = Double.NaN;
                }

                m_turretMotor.set(TalonSRXControlMode.PercentOutput, 0.3 * (reverse ? -1 : 1));
            }),
            Commands.waitSeconds(0.1),
            Commands.waitUntil(() -> m_turretMotor.getSelectedSensorVelocity() == 0),
            this.runOnce(this::disableTurret),
            this.runOnce(() -> {
                if (reverse) {
                    m_minEncoderValue = m_turretMotor.getSelectedSensorPosition();
                    m_turretMotor.configReverseSoftLimitEnable(true);
                    m_turretMotor.configReverseSoftLimitThreshold(m_minEncoderValue + kSoftLimitThreshhold);
                } else {
                    m_maxEncoderValue = m_turretMotor.getSelectedSensorPosition();
                    m_turretMotor.configForwardSoftLimitEnable(true);
                    m_turretMotor.configForwardSoftLimitThreshold(m_maxEncoderValue - kSoftLimitThreshhold);
                }
            })
        ).finallyDo(this::disableTurret);
    }

    public boolean isTurretCalibrated() {
        // The limits will be NaN when not calibrated
        return !(Double.isNaN(m_minEncoderValue) || Double.isNaN(m_maxEncoderValue));
    }
    
    public Command findTurretLimitsCommand() {
        return Commands.sequence(
            testTurretLimit(false),
            testTurretLimit(true)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder pos (raw)", m_turretMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Encoder pos", getPosition());
        SmartDashboard.putNumber("Encoder max", m_maxEncoderValue);
        SmartDashboard.putNumber("Encoder min", m_minEncoderValue);
        SmartDashboard.putNumber("Encoder range", Math.abs(m_maxEncoderValue - m_minEncoderValue));
    }
}
