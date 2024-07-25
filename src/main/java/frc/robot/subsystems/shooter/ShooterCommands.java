package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Hopper.HopperMotorState;

public class ShooterCommands {
  private final Flywheel m_flywheel;
  private final Hopper m_hopper;
  private final IntakeSubsystem m_intake;

  public ShooterCommands(Hopper hopper, Flywheel flywheel, IntakeSubsystem intake) {
    m_flywheel = flywheel;
    m_hopper = hopper;
    m_intake = intake;
  }

  public Command primeBallForShooting() {
    return Commands.deadline(
        Commands.sequence(
            Commands.runOnce(() -> m_hopper.set(HopperMotorState.RUNNING), m_hopper),
            Commands.waitUntil(m_hopper::isPrimed)),
        m_intake.getPassiveAgitateCommand())
        .finallyDo(() -> m_hopper.set(HopperMotorState.STOPPED));
  }

  public Command primeFlywheel() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          m_flywheel.setVelocityRaw(30000);
        }, m_flywheel),
        Commands.waitSeconds(0.5),
        Commands.waitUntil(m_flywheel::isAtTargetVelocity));
  }

  private Command shoot() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          m_hopper.set(HopperMotorState.SHOOTING);
        }, m_hopper),
        Commands.parallel(
            m_intake.getPassiveAgitateCommand(),
            Commands.run(() -> {
              // TODO: Set speed and turret angle based on vision
            }, m_flywheel)))
        .finallyDo(() -> {
          m_hopper.set(HopperMotorState.STOPPED);
          m_flywheel.stop();
        });
  }

  /**
   * Continues shooting until command is canceled
   */
  public Command shootContinuous() {
    return Commands.sequence(
        primeFlywheel(),
        shoot());
  }

  /**
   * Shoots until no more balls are detected by hopper sensors
   */
  public Command shootAll() {
    Debouncer hopperDebouncer = new Debouncer(0.75);
    return Commands.sequence(
        primeFlywheel(),
        Commands.deadline(
            Commands.sequence(
                Commands.waitSeconds(0.3),
                // Reset the debouncer
                Commands.runOnce(() -> hopperDebouncer.calculate(false)),
                Commands.waitUntil(() -> hopperDebouncer.calculate(!m_hopper.hasAnyBalls()) && !m_flywheel.hasBallInHood())),
            shoot()));
  }
}
