// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Ids;
import frc.robot.auto.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climbing.ClimbingSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.SimpleVision;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);

  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  private final Drivetrain m_drive = new Drivetrain(TunerConstants.DriveTrain);

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final Flywheel m_flywheel = new Flywheel();
  private final ShooterCommands m_shooter = new ShooterCommands(new Hopper(), m_flywheel, m_intake);
  private final ClimbingSubsystem m_climbers = new ClimbingSubsystem();
  private final Compressor m_compressor = new Compressor(Ids.kPneumaticControllerCanId, PneumaticsModuleType.CTREPCM);
  private final SimpleVision m_vision = new SimpleVision();
  private final TurretSubsystem m_turret = new TurretSubsystem();

  private final Autos m_autos = new Autos(m_drivetrain);

  private void configureBindings() {
    // Driving with standard controls
    m_drive.applyDefault(
        () -> m_drive.driveFieldCentric(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX()));

    joystick.a().whileTrue(m_drive.applyBrake());
    joystick.b().whileTrue(m_drive.apply(() -> m_drive.point(-joystick.getLeftY(), -joystick.getLeftX())));
    joystick.leftTrigger().whileTrue(m_drivetrain.run(() -> m_vision.aimAtTag(15, m_drivetrain)));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(m_drive.reorientForward());

    // Gamepiece handling
    joystick.x().whileTrue(m_intake.getIntakeCommand());
    joystick.y().whileTrue(m_shooter.primeBallForShooting());
    joystick.rightTrigger().whileTrue(m_shooter.shootContinuous());
    joystick.rightBumper().onTrue(m_shooter.shootAll());

    // Climbing
    joystick.povUp().onTrue(m_climbers.runOnce(m_climbers::raiseArms));
    joystick.povDown().onTrue(m_climbers.runOnce(m_climbers::lowerArms));

    // Turret
    m_turret.setDefaultCommand(m_turret.run(() -> m_turret.setOutput(-joystick2.getLeftX() * 0.4)));
    joystick2.b().whileTrue(m_turret.findTurretLimitsCommand());
    joystick2.x().whileTrue(
        m_turret.runEnd(() -> m_turret.setPosition(Rotation2d.fromDegrees(SmartDashboard.getNumber("pos", 0))), () -> m_turret.setOutput(0)));
  }

  public RobotContainer() {
    m_compressor.enableDigital();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autos.shootIntakeTest(m_intake, m_shooter);
    return Commands.sequence(
        //TODO Reset to Zero? Gotta do something so it doesn't shoot in the wrong direction
        m_turret.findTurretLimitsCommand(),
        autoCommand);
  }
}
