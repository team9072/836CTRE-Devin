package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeMotor.IntakeMotorState;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeMotor m_motors = new IntakeMotor();
    private final IntakePneumatics m_pneumatics = new IntakePneumatics();

    private void deploy() {
        m_pneumatics.deploy();
        m_motors.set(IntakeMotorState.INTAKE);
    }

    private void retract() {
        m_pneumatics.retract();
        m_motors.set(IntakeMotorState.STOPPED);
    }

    /**
     * Slowly rotates the intake while retracted to free stuck balls in the hopper.
     * Keeps going until command is cancelled
     */
    public Command getAgitateComamnd() {
        return this.startEnd(() -> {
            m_pneumatics.retract();
            m_motors.set(IntakeMotorState.AGITATE);
        }, this::retract);
    }

    /**
     * Runs the agitate command as long as no other command is running on the intake subsystem
     * and cancels it when this command ends.
     * The agitate command will be rescheduled if an interupting command finishes.
     */
    public Command getPassiveAgitateCommand() {
        Command agitateComamnd = getAgitateComamnd();
        return Commands.runEnd(() -> {
            if (this.getCurrentCommand() == null) {
                agitateComamnd.schedule();
            }
        }, () -> agitateComamnd.cancel());
    }

    /**
     * Deploys the intake while the command runs and retracts it when cancelled
     */
    public Command getIntakeCommand() {
        return this.startEnd(this::deploy, this::retract);
    }

    public boolean isDeployed() {
        return m_pneumatics.isDeployed();
    }
}
