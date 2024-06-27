package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Hopper.HopperMotorState;

public class ShooterCommands {
    private final Flywheel m_flywheel;
    private final Hopper m_hopper;

    public ShooterCommands(Hopper hopper, Flywheel flywheel) {
        m_flywheel = flywheel;
        m_hopper = hopper;
    }

    public Command primeShooter() {
        return Commands.sequence(
                Commands.runOnce(() -> m_hopper.set(HopperMotorState.RUNNING), m_hopper),
                Commands.waitUntil(m_hopper::isPrimed)).finallyDo(() -> m_hopper.set(HopperMotorState.STOPPED));
    }

    public Command shootContinuous() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                    m_hopper.set(HopperMotorState.STOPPED);
                    m_flywheel.setVelocityRaw(30000);
                }, m_hopper, m_flywheel),
            Commands.waitSeconds(0.5),
            Commands.waitUntil(m_flywheel::isAtTargetVelocity),
            Commands.runOnce(() -> m_hopper.set(HopperMotorState.SHOOTING), m_hopper),
            Commands.run(() -> {
                // TODO: Set speed and turret angle based on vision
            }, m_flywheel)
            ).finallyDo(() -> {
                m_hopper.set(HopperMotorState.STOPPED);
                m_flywheel.stop();
            });
    }
}
