package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.Base.SimpleControllerConstants;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import frc.robot.RobotState;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Map;

public class SubsystemConstants {
    public static final ControllerConstants kExampleSubsystem = new ControllerConstants();
    static {
        /* Base */
        kExampleSubsystem.real.base.main.id = 30;
        kExampleSubsystem.real.base.main.inverted = true;
        kExampleSubsystem.real.base.followers = new SimpleControllerConstants[] { new SimpleControllerConstants(), new SimpleControllerConstants() };
        kExampleSubsystem.real.base.followers[0].id = 31;
        kExampleSubsystem.real.base.followers[0].inverted = false;
        kExampleSubsystem.real.base.followers[1].id = 32;
        kExampleSubsystem.real.base.followers[1].inverted = false;
        kExampleSubsystem.real.base.isBrakeMode = false;

        /* Control */
        kExampleSubsystem.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, Double.POSITIVE_INFINITY, 0.1175, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kExampleSubsystem.real.control.velocityGoalTolerance = 6;
        kExampleSubsystem.real.control.enableFOC = true;

        /* Simulation */
        kExampleSubsystem.simMotor = DCMotor.getKrakenX60(2);
        kExampleSubsystem.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (1000 * 2 * Math.PI / 0.5));
    }



    public static final SwerveConstants kSwerve = new SwerveConstants();
    static {
        /* Chassis */
        kSwerve.chassis.trackWidth = 0.59475;
        kSwerve.chassis.wheelBase = 0.51855;
        kSwerve.chassis.bumperLength = 0.8079;
        kSwerve.chassis.bumperWidth = 0.8841;
        kSwerve.chassis.kinematics = new SwerveDriveKinematics(
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0)
        );

        /* Limits */
        kSwerve.limits.maxSpeed = GeneralConstants.kRobotMode.isSim() ? 5.145 : 4.7;
        kSwerve.limits.maxAngularVelocity = 8.5;
        kSwerve.limits.speedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationSpeedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationAccelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.maxSkidAcceleration = 60;
        kSwerve.limits.maxForwardAcceleration = 20;
        kSwerve.limits.discretizeFactor = 2.75;

        /* Modules */
        double wheelRadius = 0.049;
        kSwerve.modules.openLoop = GeneralConstants.kRobotMode.isSim();
        kSwerve.modules.driveMotorConstants = new ControllerConstants();
        kSwerve.modules.driveMotorConstants.real.base.statorCurrentLimit = 100;
        kSwerve.modules.driveMotorConstants.real.base.supplyCurrentLimit = 60;
        kSwerve.modules.driveMotorConstants.real.control.gearRatio = 5.9;
        kSwerve.modules.driveMotorConstants.real.control.conversionFactor = wheelRadius * 2 * Math.PI;
        kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createPIDF(1, 0, 0, 0.5, 2.35, 0, 0.3, 0, GravityTypeValue.Elevator_Static);

        kSwerve.modules.steerMotorConstants = new ControllerConstants();
        kSwerve.modules.steerMotorConstants.real.base.statorCurrentLimit = 40;
        kSwerve.modules.steerMotorConstants.real.base.supplyCurrentLimit = 30;
        kSwerve.modules.steerMotorConstants.real.control.gearRatio = 18.75;
        kSwerve.modules.steerMotorConstants.real.control.conversionFactor = 2 * Math.PI;
        kSwerve.modules.steerMotorConstants.real.control.controlConstants = ControlConstants.createPID(25, 30, 0.25, Math.PI);

        kSwerve.modules.driveControllerType = Controller.ControllerType.TalonFX;
        kSwerve.modules.steerControllerType = Controller.ControllerType.TalonFX;
        kSwerve.modules.moduleConstants = new SwerveModuleConstants[4];

        for (int i = 0; i < 4; i++) {
            kSwerve.modules.moduleConstants[i] = new SwerveModuleConstants(
                i,
                10 + i * 2,
                11 + i * 2,
                false,
                false,
                6 + i,
                false,
                0
            );
        }

        kSwerve.modules.moduleConstants[0].CANCoderOffset = -0.293213;
        kSwerve.modules.moduleConstants[1].CANCoderOffset = -0.470947;
        kSwerve.modules.moduleConstants[2].CANCoderOffset = -0.270996;
        kSwerve.modules.moduleConstants[3].CANCoderOffset = 0.481934;

        /* Gyro */
        kSwerve.gyro.gyroID = 5;
        kSwerve.gyro.gyroInverted = false;
        kSwerve.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

        /* Simulation */
        kSwerve.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerve.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerve.simulation.swerveType = SwerveConstants.Simulation.SwerveType.Mark4n;
        kSwerve.simulation.gearRatioLevel = 2;

        /* Special */
        kSwerve.special.enableOdometryThread = false;
        kSwerve.special.odometryThreadFrequency = 50;
        kSwerve.special.isReplay = GeneralConstants.kRobotMode.isReplay();
        kSwerve.special.robotStartPose = new Pose2d(2, 4, Rotation2d.kZero);
        kSwerve.special.CANBus = new CANBus("Swerve Bus");
        kSwerve.special.enableAutoLock = false;

        try {
            kSwerve.special.robotConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }



    public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants();
    static {
        kSwerveController.swerveConstants = kSwerve;
        kSwerveController.drivePIDConstants = ControlConstants.createPID(6.5, 0, 0, 0);
        kSwerveController.rotationPIDConstants = ControlConstants.createPID(5.5, 0, 0, 0);
        kSwerveController.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
    }



    public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
        new PIDConstants(3.5, 0, 0),
        new PIDConstants(3.5, 0, 0)
    );



    public static final VisionConstants kVision = new VisionConstants();
    static {
        kVision.cameras = Map.of(
            "limelight-front", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight),
            "limelight-right", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight)
        );

        kVision.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;
        kVision.isReplay = GeneralConstants.kRobotMode.isReplay();
        kVision.robotPoseSupplier = () -> RobotState.get().getRobotPose();
    }
}
