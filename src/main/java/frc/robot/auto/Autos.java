package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.ChoreoDriver.TrajectoryDriver;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterCommands;

public class Autos {
    private final ChoreoDriver m_driver;

    public Autos(CommandSwerveDrivetrain drivetrain) {
        m_driver = new ChoreoDriver(drivetrain);
    }

    public Command shootIntakeTest(IntakeSubsystem intake, ShooterCommands shooter) {
        TrajectoryDriver shootTraj = m_driver.loadTrajectory("Shoot and get object test");

        return Commands.sequence(
                shootTraj.start(),
                shooter.shootAll(),
                Commands.deadline(
                        shootTraj.next(),
                        intake.getIntakeCommand()),
                shooter.shootAll(),
                shootTraj.next());
    }
}
