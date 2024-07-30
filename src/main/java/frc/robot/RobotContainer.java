// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Ids;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climbing.ClimbingSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.SimpleVision;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed); 

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final Flywheel m_flywheel = new Flywheel();
  private final ShooterCommands m_shooter = new ShooterCommands(new Hopper(), m_flywheel, m_intake);
  private final ClimbingSubsystem m_climbers = new ClimbingSubsystem();
  private final Compressor m_compressor = new Compressor(Ids.kPneumaticControllerCanId, PneumaticsModuleType.CTREPCM);
  private final SimpleVision m_vision = new SimpleVision();
  private final TurretSubsystem m_turret = new TurretSubsystem();

  private void configureBindings() {
    // Driving
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    // Gamepiece handling
    joystick.x().whileTrue(m_intake.getIntakeCommand());
    joystick.y().whileTrue(m_shooter.primeBallForShooting());
    joystick.rightTrigger().whileTrue(m_shooter.shootContinuous());
    joystick.rightBumper().onTrue(m_shooter.shootAll());

    joystick.leftTrigger().whileTrue(Commands.run(() -> m_vision.aimAtTag(15, drivetrain), drivetrain));

    // Climbing
    joystick.povUp().onTrue(Commands.runOnce(m_climbers::raiseArms, m_climbers));
    joystick.povDown().onTrue(Commands.runOnce(m_climbers::lowerArms, m_climbers));

    // Turret
    joystick2.b().whileTrue(m_turret.findTurretLimitsCommand());
    m_turret.setDefaultCommand(Commands.run(() -> m_turret.setOutput(-joystick2.getLeftX() * 0.4), m_turret));
    joystick2.x().whileTrue(Commands.runEnd(() -> m_turret.setPosition(SmartDashboard.getNumber("pos", 0)), () -> m_turret.setOutput(0), m_turret));
  }

  public RobotContainer() {
    m_compressor.enableDigital();
    configureBindings();
  }

  public Command getChoreoCommand(ChoreoTrajectory traj) {
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();
    return Choreo.choreoSwerveCommand(
      traj,
      () -> drivetrain.getState().Pose,
      Choreo.choreoSwerveController(
        new PIDController(5, 0, 0),
        new PIDController(5, 0, 0),
        new PIDController(5, 0, 0)
      ),
      (speeds) -> drivetrain.setControl(request.withSpeeds(speeds)),
      () -> false,
      drivetrain
    );
  }

  public Command getAutonomousCommand() {
    var trajParts = Choreo.getTrajectoryGroup("Shoot and get object test");
    drivetrain.seedFieldRelative(trajParts.get(0).getInitialState().getPose());

    return Commands.sequence(
      getChoreoCommand(trajParts.get(0)),
      m_shooter.shootAll(),
      Commands.deadline(
        getChoreoCommand(trajParts.get(1)),
        m_intake.getIntakeCommand()  
      ),
      m_shooter.shootAll(),
      getChoreoCommand(trajParts.get(2))
    );
  }
}
