package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.localization.RobotPose;
import frc.lib.NinjasLib.localization.VisionFiltersCalculator;
import frc.lib.NinjasLib.localization.VisionStrengthCalculator;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.robot.Robot;
import frc.robot.constants.SubsystemConstants;

public class VisionSubsystem extends SubsystemBase {
    private final boolean enabled;
    private int framesSinceGyroUpdate = 75;
    private boolean isFirstDisable = true;
    private boolean resettedGyro = false;

    private Pose2d megaTag1Pose = new Pose2d();
    private double megaTag1DistFromTag;

    private Pose2d lastVisionPose = new Pose2d();

    public final VisionFiltersCalculator kFilters = visionOutput -> {
        SwerveSpeeds speed = frc.lib.NinjasLib.swerve.Swerve.get().getSpeeds();

        return ((Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) <= SubsystemConstants.Vision.kMaxSpeedFilter
            && Math.abs(speed.omegaRadiansPerSecond) <=SubsystemConstants.Vision.kMaxAngularSpeedFilter
            && visionOutput.closestTargetDist <= SubsystemConstants.Vision.kMaxDistanceFilter
            && visionOutput.closestTargetDist >= SubsystemConstants.Vision.kMinDistanceFilter
            && visionOutput.ambiguity <= SubsystemConstants.Vision.kMaxAmbiguityFilter)
            || DriverStation.isDisabled())
            && resettedGyro;
    };

    public final VisionStrengthCalculator kStrength = VisionStrengthCalculator.kDefault;

    public VisionSubsystem(boolean enabled) {
        this.enabled = enabled;

        RobotPose.setInstance(new RobotPose(SubsystemConstants.Swerve.kSwerve.chassis.kinematics, kStrength, kFilters));
        if (enabled) {
            Vision.setInstance(new Vision(SubsystemConstants.Vision.kVision));
        }
    }

    public void periodic() {
        if (!enabled)
            return;

        Vision.get().periodic();

        // Reset gyro every 75 loops (1.5s) only if the robot is disabled and NOT in transition between auto to teleop
        if (DriverStation.isEnabled()) {
            isFirstDisable = !DriverStation.isAutonomous();
        } else if (isFirstDisable && DriverStation.isAutonomous() && !Robot.isSimulation()) {
            framesSinceGyroUpdate++;
            if (framesSinceGyroUpdate >= 75 && megaTag1Pose != null) {
                RobotPose.get().resetGyro(megaTag1Pose.getRotation());
                resettedGyro = true;
                framesSinceGyroUpdate = 0;
            }
        }

        megaTag1Pose = null;
        megaTag1DistFromTag = 0;
        VisionOutput[] outputs = Vision.get().getVisionOutputs();
        for (VisionOutput output : outputs) {
            if(!output.hasTargets)
                continue;

            boolean passedFilters = kFilters.isPassed(output);
//            Matrix<N3, N1> strength = kStrength.calculate(output);

            if (output.hasTargetsMegaTag1) {
                megaTag1Pose = output.robotPoseMegaTag1;
                megaTag1DistFromTag = output.closestTargetDistMegaTag1;
            }
            RobotPose.get().addVisionUpdate(output, output.timestamp);

            NinjasLogger.log("Vision/" + output.cameraName + "/Vision Pose", output.robotPose);
            if (passedFilters) {
                NinjasLogger.log("Vision/" + output.cameraName + "/Vision Pose (Passed Filters)", output.robotPose);
                lastVisionPose = output.robotPose;
            }
        }

        NinjasLogger.log("Vision/MegaTag 1 Vision", megaTag1Pose);
        NinjasLogger.log("Vision/MegaTag 1 Vision Dist", megaTag1DistFromTag);
        NinjasLogger.log("Vision/Odometry Only Pose", RobotPose.get().getOdometryOnlyRobotPose());
        NinjasLogger.log("Vision/Odometry Vision Error", lastVisionPose.getTranslation().getDistance(RobotPose.get().getOdometryOnlyRobotPose().getTranslation()));
    }
}
