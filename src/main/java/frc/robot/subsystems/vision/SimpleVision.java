package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class SimpleVision {
    private static final PhotonCamera camera = new PhotonCamera("BW3");

    private final SwerveRequest.RobotCentric aiming = new SwerveRequest.RobotCentric();
    private final PIDController m_robotAimController = new PIDController(0.1, 0, 0);

    public Optional<PhotonTrackedTarget> getAprilTag(int tagId) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        // Could get messy if 2 tags with same id are seen, but that shouln't happen in
        // normal circumstances
        return result.getTargets()
                .stream()
                .filter(t -> (t.getFiducialId() == tagId))
                .findAny();
    }

    private SwerveRequest.RobotCentric getRobotAimingRequest(int tagId) {
        Optional<PhotonTrackedTarget> potentialTag = getAprilTag(tagId);
        if (potentialTag.isEmpty()) {
            return aiming.withVelocityY(0)
                    .withRotationalRate(0);
        }

        PhotonTrackedTarget tag = potentialTag.get();

        double feedback = m_robotAimController.calculate(tag.getYaw(), 0);

        return aiming.withRotationalRate(feedback);
    }

    public Command aimRobotAtTag(int tagId, CommandSwerveDrivetrain drivetrain) {
        return drivetrain.applyRequest(() -> getRobotAimingRequest(tagId));
    }

    public Command aimTurretAtTag(int tagId, TurretSubsystem turret, SwerveDrivetrain drivetrain) {
        return Commands.run(() -> {
            Optional<PhotonTrackedTarget> potentialTag = getAprilTag(tagId);
            if (potentialTag.isEmpty()) {
                turret.setOutput(0);
                return;
            }

            PhotonTrackedTarget tag = potentialTag.get();

            turret.setPosition(turret.getPosition().orElse(new Rotation2d()).minus(Rotation2d.fromDegrees(tag.getYaw())));
        }, turret);
    }
}
