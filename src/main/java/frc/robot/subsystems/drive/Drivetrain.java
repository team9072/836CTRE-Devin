package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

/**
 * Convinience class to wrap the CommandSwerveDrivetrain for control via joysticks
 */
public class Drivetrain {
    private static final double kMaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private static final double kMaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentric fieldDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.1);
    
    private final SwerveRequest.RobotCentric robotDriveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(kMaxSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.1);

    public final CommandSwerveDrivetrain m_drivetrain;
    private final Telemetry logger = new Telemetry(kMaxSpeed);

    public Drivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_drivetrain.registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for field-relative maneuvers.
     */
    public Command reorientForward() {
        return Commands.runOnce(() -> m_drivetrain.seedFieldRelative());
    }

    /**
     * Command to control the drivetrain with the specified request
     */
    public Command apply(Supplier<SwerveRequest> requestSupplier) {
        return m_drivetrain.applyRequest(requestSupplier);
    }

    /**
     * Sets the default drivetrain command to apply the specified request
     */
    public void applyDefault(Supplier<SwerveRequest> requestSupplier) {
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(requestSupplier));
    }

    /**
     * Applies the brake request
     */
    public Command applyBrake() {
        return apply(this::brake);
    }

    /**
     * Request to brake by turning the wheels in an "X" pattern
     */
    public SwerveRequest.SwerveDriveBrake brake() {
        return brakeRequest;
    }

    /**
     * Request to point the wheels at a certain direction
     */
    public SwerveRequest.PointWheelsAt point(Rotation2d rotation) {
        return pointRequest.withModuleDirection(rotation);
    }

    /**
     * Request to point the wheels in a direction created from x and y components
     */
    public SwerveRequest.PointWheelsAt point(double x, double y) {
        return pointRequest.withModuleDirection(new Rotation2d(x, y));
    }

    /**
     * Request to drive the robot in field centric mode
     */
    public SwerveRequest.FieldCentric driveFieldCentric(double xPercent, double yPercent, double rotPercent) {
        return fieldDriveRequest.withVelocityX(xPercent * kMaxSpeed)
                .withVelocityY(yPercent * kMaxSpeed)
                .withRotationalRate(rotPercent * kMaxAngularRate);
    }

    /**
     * Request to drive the robot in robot centric mode
     */
    public SwerveRequest.RobotCentric driveRobotCentric(double xPercent, double yPercent, double rotPercent) {
        return robotDriveRequest.withVelocityX(xPercent * kMaxSpeed)
                .withVelocityY(yPercent * kMaxSpeed)
                .withRotationalRate(rotPercent * kMaxAngularRate);
    }
}
