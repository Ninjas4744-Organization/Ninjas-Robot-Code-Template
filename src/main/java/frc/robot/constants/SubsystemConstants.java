package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.Base.SimpleControllerConstants;
import frc.lib.NinjasLib.localization.RobotPose;
import frc.lib.NinjasLib.localization.vision.FieldLayoutGetter;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import frc.robot.RobotState;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Map;
import java.util.Optional;

public class SubsystemConstants {
    public static final ControllerConstants kExampleSubsystem = new ControllerConstants();
    static {
        kExampleSubsystem.real
                .withBase(new RealControllerConstants.Base()
                        .withMain(new RealControllerConstants.Base.SimpleControllerConstants(30, false))
                        .withFollowers(new RealControllerConstants.Base.SimpleControllerConstants[]{
                                new RealControllerConstants.Base.SimpleControllerConstants(31, false), new RealControllerConstants.Base.SimpleControllerConstants(31, true),
                                new RealControllerConstants.Base.SimpleControllerConstants(32, false)
                        })
                        .withIsBrakeMode(false)
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimit(80))
                .withControl(new RealControllerConstants.Control()
                        .withControlConstants(ControlConstants.createProfiledPIDF(
                                4.0, 0, 0, 0,
                                80, 160, 0,
                                0, 0, 0, 0, null))
                        .withConversion(12.0, 1)
                        .withVelocityGoalTolerance(10));
    }

    private static final double kWheelRadius = 0.049; // meters

    // Declare these *above* kSwerve: static fields initialize top to bottom, so a field that
    // is defined further down would still be null when kSwerve reads it.
    public static final ControllerConstants kDriveMotorConstants = new ControllerConstants();
    public static final ControllerConstants kSteerMotorConstants = new ControllerConstants();

    static {
        // Drive: velocity control, in meters and meters per second
        kDriveMotorConstants.real
                .withBase(new RealControllerConstants.Base()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(60))
                .withControl(new RealControllerConstants.Control()
                        .withConversion(6.75, 2 * Math.PI * kWheelRadius)     // gear ratio, circumference
                        .withControlConstants(ControlConstants.createPIDF(
                                1.0, 0, 0, 0,                                     // P, I, D, IZone
                                2.35, 0, 0.3, 0, GravityTypeValue.Elevator_Static))); // V, A, S, G, gravity type

        // Steer: position control, in radians
        kSteerMotorConstants.real
                .withBase(new RealControllerConstants.Base()
                        .withStatorCurrentLimit(40)
                        .withSupplyCurrentLimit(30))
                .withControl(new RealControllerConstants.Control()
                        .withConversion(150.0 / 7, 2 * Math.PI)               // gear ratio, radians per rotation
                        .withControlConstants(ControlConstants.createPID(25, 0, 0.25, 0)));
    }

    public static final SwerveConstants kSwerve;

    static {
        try {
            kSwerve = new SwerveConstants()
                    .withChassis(new SwerveConstants.Chassis()
                            .withDimensions(0.6, 0.6)      // track width, wheel base (meters)
                            .withBumper(0.9, 0.9))         // bumper width, length (meters)
                    .withSpeeds(new SwerveConstants.Speeds()
                            .withMaxSpeeds(5.0, 9.0)                       // physical max: m/s, rad/s
                            .withSpeedLimits(4.0, 7.0)                     // soft driving limits: m/s, rad/s
                            .withAccelerationLimits(20, 12, 15))           // skid, forward, rotation accel
                    .withModules(new SwerveConstants.Modules()
                            .withModuleConstants(new SwerveModuleConstants[] {
                                    new SwerveModuleConstants(0, 1, 2, false, true, 3, false, 0.421),  // front left
                                    new SwerveModuleConstants(1, 4, 5, false, true, 6, false, 0.128),  // front right
                                    new SwerveModuleConstants(2, 7, 8, false, true, 9, false, 0.355),  // back left
                                    new SwerveModuleConstants(3, 10, 11, false, true, 12, false, 0.009) // back right
                            })
                            .withDriveMotor(kDriveMotorConstants, Controller.ControllerType.TalonFX)
                            .withSteerMotor(kSteerMotorConstants, Controller.ControllerType.TalonFX))
                    .withGyro(new SwerveConstants.Gyro(5, false, SwerveConstants.Gyro.GyroType.Pigeon2))
                    .withSpecial(new SwerveConstants.Special()
                            .withRobotConfig(RobotConfig.fromGUISettings())
                            .withRobotStartPose(new Pose2d(3, 3, Rotation2d.kZero)));
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }


    public static final VisionConstants kVision = new VisionConstants()
            .withFieldLayoutGetter(FieldConstants::getFieldLayoutWithIgnored)
            .withRobotPoseSupplier(() -> RobotPose.get().getRobotPose())
            .withPhotonVision("FrontCam", new Transform3d(0, 0, 0, Rotation3d.kZero))
            .withLimelight("limelight-back");
}
