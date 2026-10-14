package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.Base;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.Control;
import frc.lib.NinjasLib.localization.RobotPose;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants.*;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class SubsystemConstants {
    public static final ControllerConstants kExampleSubsystem = new ControllerConstants()
        .withBase(new Base()
            .withMain(new Base.SimpleControllerConstants(30, false))
            .withFollowers(new Base.SimpleControllerConstants[] {
                new Base.SimpleControllerConstants(31, false),
                new Base.SimpleControllerConstants(32, false)
            })
            .withIsBrakeMode(false)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(80)
        ).withControl(new Control()
            .withControlConstants(ControlConstants.createPID(1, 0, 0, 0))
            .withConversion(12.0, 1)
            .withVelocityGoalTolerance(7)
        ).withSim(() -> new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(3), 0.01, 12), DCMotor.getKrakenX60(3)));

    public static class Swerve {
        public static double kDriverSpeedFactor = 1;
        public static double kDriverRotationSpeedFactor = 1;
        public static double kDriverPowFactor = 1;
        public static double kDriverRotationPowFactor = 1;
        public static double kJoystickDeadband = 0.04;
        public static boolean kDriverFieldRelative = true;

        private static final double kWheelRadius = 0.049; // meters
        private static final RobotConfig robotConfig;

        static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException e) {
                throw new RuntimeException(e);
            } catch (ParseException e) {
                throw new RuntimeException(e);
            }
        }

        private static final ControllerConstants kDriveMotor = new ControllerConstants()
            .withBase(new Base()
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(60)
            ).withControl(new Control()
                .withConversion(6.75, 2 * Math.PI * kWheelRadius)
                .withControlConstants(ControlConstants.createPIDF(1, 0, 0, 0.5, 2.35, 0, 0.3, 0, GravityTypeValue.Elevator_Static))
            );

        private static final ControllerConstants kSteerMotor = new ControllerConstants()
            .withBase(new Base()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40)
            ).withControl(new Control()
                .withConversion(18.75, 2 * Math.PI)
                .withControlConstants(ControlConstants.createPID(25, 30, 0.25, Math.PI))
            );

        public static final SwerveConstants kSwerve = new SwerveConstants()
            .withChassis(new Chassis()
                .withDimensions(0.6, 0.6)
                .withBumper(0.8, 0.8)
            )
            .withSpeeds(new Speeds()
                .withMaxSpeeds(4.0, 9.0)
                .withSpeedLimits(4.0, 9.0)
                .withAccelerationLimits(60, 20, Double.POSITIVE_INFINITY)
                .withDiscretizeFactor(2.75)
            )
            .withModules(new Modules()
                .withModuleConstants(new SwerveModuleConstants[] {
                    new SwerveModuleConstants(0, 10, 11, false, true, 6, false, 0.421), // front left
                    new SwerveModuleConstants(1, 12, 13, false, true, 7, false, 0.128), // front right
                    new SwerveModuleConstants(2, 14, 15, false, true, 8, false, 0.355), // back left
                    new SwerveModuleConstants(3, 16, 17, false, true, 9, false, 0.009)  // back right
                })
                .withOpenLoop(false)
                .withDriveMotor(kDriveMotor, Controller.ControllerType.TalonFX)
                .withSteerMotor(kSteerMotor, Controller.ControllerType.TalonFX)
            )
            .withGyro(new Gyro(5, false, Gyro.GyroType.Pigeon2))
            .withSpecial(new Special()
                .withRobotConfig(robotConfig)
                .withRobotStartPose(new Pose2d(3, 3, Rotation2d.kZero))
                .withCANBus(new CANBus("swerve"))
            );

        public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants()
            .withSwerveConstants(kSwerve)
            .withDrivePID(ControlConstants.createPID(6.5, 0, 0, 0))
            .withRotationPID(ControlConstants.createPID(5.5, 0, 0, 0));

        public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
            new PIDConstants(3.5, 0, 0),
            new PIDConstants(3.5, 0, 0)
        );
    }

    public static class Vision {
        public static final double kMaxDistanceFilter = 5;
        public static final double kMinDistanceFilter = 0.6;
        public static final double kMaxSpeedFilter = 3;
        public static final double kMaxAngularSpeedFilter = 3;
        public static final double kMaxAmbiguityFilter = 0.2;
        public static final double kOdometryDriftPerMeter = 0.02316;

        public static final VisionConstants kVision = new VisionConstants()
            .withFieldLayoutGetter(FieldConstants::getFieldLayoutWithIgnored)
            .withRobotPoseSupplier(RobotPose.get()::getRobotPose)
            .withLimelight("limelight");
    }
}
