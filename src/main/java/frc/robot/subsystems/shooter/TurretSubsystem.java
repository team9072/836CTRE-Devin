package frc.robot.subsystems.shooter;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ids;

public class TurretSubsystem extends SubsystemBase {
    private static final double kSoftLimitThreshhold = 4487;
    private static final double kMaxToForwardOffset = 7140;

    private static final double kSensorUnitsFor180 = 16080;

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

        setDefaultCommand(this.runOnce(this::disableTurret));
    }

    public void setOutput(double percent) {
        if (!isTurretCalibrated()) {
            disableTurret();
            return;
        }

        m_turretMotor.set(TalonSRXControlMode.PercentOutput, percent);
    }

    private double getPositionRaw() {
        if (!isTurretCalibrated()) return Double.NaN;
        return m_turretMotor.getSelectedSensorPosition() + (kMaxToForwardOffset - m_maxEncoderValue);
    }
    
    public Optional<Rotation2d> getPosition() {
        if (!isTurretCalibrated()) return Optional.empty();
        return Optional.of(Rotation2d.fromRotations(getPositionRaw() / (2 * kSensorUnitsFor180)));
    }

    private void setPositionRaw(double position) {
        if (!isTurretCalibrated()) {
            disableTurret();
            return;
        }

        m_turretMotor.set(TalonSRXControlMode.Position, Math.max(getMinLimit(), Math.min(position - (kMaxToForwardOffset - m_maxEncoderValue), getMaxLimit())));
    }

    public void setPosition(Rotation2d position) {
        double rotations = position.getRotations();
        // normalize position to 0-1 rotations
        double wrappedRotations = (rotations % 1 + 1) % 1;
        // go clockwise unless position is inaccessible due to limits
        double adjustedRotations = wrappedRotations < 0.25 ? wrappedRotations : wrappedRotations - 1;

        setPositionRaw(adjustedRotations * kSensorUnitsFor180 * 2);
    }

    private void disableTurret() {
        m_turretMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    private double getMaxLimit() {
        return m_maxEncoderValue - kSoftLimitThreshhold;
    }

    private double getMinLimit() {
        return m_minEncoderValue + kSoftLimitThreshhold;
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
                    m_turretMotor.configReverseSoftLimitThreshold(getMinLimit());
                } else {
                    m_maxEncoderValue = m_turretMotor.getSelectedSensorPosition();
                    m_turretMotor.configForwardSoftLimitEnable(true);
                    m_turretMotor.configForwardSoftLimitThreshold(getMaxLimit());
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
        SmartDashboard.putNumber("Encoder pos", getPosition().orElse(new Rotation2d()).getDegrees());
        SmartDashboard.putNumber("Encoder max", m_maxEncoderValue);
        SmartDashboard.putNumber("Encoder min", m_minEncoderValue);
        SmartDashboard.putNumber("Encoder range", Math.abs(m_maxEncoderValue - m_minEncoderValue));
    }
}
