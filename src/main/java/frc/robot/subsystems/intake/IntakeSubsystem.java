package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeMotor.IntakeMotorState;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeMotor m_motors = new IntakeMotor();
    private final IntakePneumatics m_pneumatics = new IntakePneumatics();

    public void deploy() {
        m_pneumatics.deploy();
        m_motors.set(IntakeMotorState.INTAKE);
    }

    public void retract() {
        m_pneumatics.retract();
        m_motors.set(IntakeMotorState.STOPPED);
    }

    public Command getIntakeCommand() {
        return Commands.startEnd(this::deploy, this::retract, this);
    }
}
