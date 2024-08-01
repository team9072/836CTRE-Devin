package frc.robot.auto;

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class ChoreoDriver {
    private final CommandSwerveDrivetrain m_drivetrain;

    public ChoreoDriver(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    public TrajectoryDriver loadTrajectory(String trajName) {
        return new TrajectoryDriver(trajName);
    }

    public class TrajectoryDriver {
        private ArrayList<ChoreoTrajectory> m_trajectories;
        private int currentTrajectory = -1;

        private TrajectoryDriver(String trajName) {
            m_trajectories = Choreo.getTrajectoryGroup(trajName);
        }

        public Command start(int stopPoint) {
            currentTrajectory = stopPoint;
            m_drivetrain.seedFieldRelative(m_trajectories.get(currentTrajectory).getInitialState().getPose());
            return next();
        }

        public Command start() {
            return start(0);
        }
        
        public Command next() {
            Command followTrajectoryCommand = new ChoreoSwerveFollower(m_trajectories.get(currentTrajectory));
            currentTrajectory++;

            return followTrajectoryCommand;
        }
    }

    private class ChoreoSwerveFollower extends Command {
        private final SwerveRequest.ApplyChassisSpeeds m_request = new SwerveRequest.ApplyChassisSpeeds();
        private final ChoreoControlFunction m_controller = Choreo.choreoSwerveController(
                        new PIDController(5, 0, 0),
                        new PIDController(5, 0, 0),
                        new PIDController(5, 0, 0));

        private final ChoreoTrajectory m_trajectory;
        private final Timer m_timer = new Timer();

        private ChoreoSwerveFollower(ChoreoTrajectory trajectory) {
            addRequirements(m_drivetrain);

            m_trajectory = trajectory;
        }

        private void setSpeeds(ChassisSpeeds speeds) {
            m_drivetrain.setControl(m_request.withSpeeds(speeds));
        }

        @Override
        public void initialize() {
            m_timer.restart();
        }

        @Override
        public void execute() {
            ChoreoTrajectoryState expectedPose = m_trajectory.sample(m_timer.get());
            Pose2d currentPose = m_drivetrain.getState().Pose;
            setSpeeds(m_controller.apply(currentPose, expectedPose));
        }

        @Override
        public void end(boolean interrupted) {
            m_timer.stop();

            if (interrupted) {
                setSpeeds(new ChassisSpeeds());
            } else {
                setSpeeds(m_trajectory.getFinalState().getChassisSpeeds());
            }
        }

        @Override
        public boolean isFinished() {
            return m_timer.hasElapsed(m_trajectory.getTotalTime());
        }
    }
}
