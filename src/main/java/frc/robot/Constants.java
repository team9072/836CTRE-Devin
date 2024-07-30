package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Ids {
        // Hopper
        public static final int kHopperMotorCanId = 31;
        public static final int kPrimeBallSensorDioId = 2;
        public static final int kInnerBallSensorDioId = 0;
        public static final int kOuterBallSensorDioId = 1;


        // Flywheel
        public static final int kFlywheelMasterMotorCanId = 21;
        public static final int kFlywheelFollowerMotorCanId = 22;
        public static final int kFlywheelBallSensorDioID = 3;

        // Intake
        public static final int kIntakeMotorCanId = 30;
        public static final int kIntakeSolenoidChannel = 0;

        // Climbing
        public static final int kClimberArmsSolenoidChannel = 1;

        // Pneumatics
        public static final int kPneumaticControllerCanId = 15;

        // Turret
        public static final int kTurretMotorCanID = 23;
    }

    // Sensors
    public static final double kBeamBreakDebounceTime = 0.05;

    // Photonvision
    public static final double kCameraHeight = Units.inchesToMeters(20.75);
    public static final double kCamraPitchRadians = Units.degreesToRadians(27);

}
