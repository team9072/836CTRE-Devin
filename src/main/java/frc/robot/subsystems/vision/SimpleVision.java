package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;

public class SimpleVision {
    private static final PhotonCamera camera = new PhotonCamera("BW3");

    private final SwerveRequest.RobotCentric aiming = new SwerveRequest.RobotCentric();
    private final PIDController m_turnController = new PIDController(0.1, 0, 0);
    
    public Optional<PhotonTrackedTarget> getAprilTag(int tagId) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        
        // Could get messy if 2 tags with same id are seen, but that shouln't happen in normal circumstances
        return result.getTargets()
            .stream()
            .filter(t -> (t.getFiducialId() == tagId))
            .findAny(); 
    }

    private SwerveRequest.RobotCentric getAimingRequest(int tagId) {
        Optional<PhotonTrackedTarget> potentialTag = getAprilTag(tagId);
        if (potentialTag.isEmpty()) {
            return aiming.withVelocityY(0)
                .withRotationalRate(0);
        };
        
        PhotonTrackedTarget tag = potentialTag.get();

        double feedback = m_turnController.calculate(tag.getYaw(), 0);

        return aiming.withRotationalRate(feedback)
            .withVelocityY(feedback);
    }

    public void aimAtTag(int tagId, SwerveDrivetrain drivetrain) {
        drivetrain.setControl(getAimingRequest(tagId));
    }
}
