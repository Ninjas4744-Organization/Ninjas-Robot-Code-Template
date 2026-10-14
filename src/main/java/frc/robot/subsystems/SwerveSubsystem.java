package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.constants.SubsystemConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final boolean enabled;

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(SubsystemConstants.Swerve.kSwerve));
            SwerveController.setInstance(new SwerveController(SubsystemConstants.Swerve.kSwerveController));
        }
    }

    public void drive(double leftX, double leftY, double rightX, double rightY) {
        if (!enabled)
            return;

        double deadband = SubsystemConstants.Swerve.kJoystickDeadband;
        double maxSpeed = SubsystemConstants.Swerve.kSwerve.speeds.maxSpeed;
        double maxAngularVelocity = SubsystemConstants.Swerve.kSwerve.speeds.maxAngularVelocity;

        Translation2d input = new Translation2d(
            -MathUtil.applyDeadband(leftY, deadband),
            -MathUtil.applyDeadband(leftX, deadband)
        );

        Rotation2d direction = input.equals(new Translation2d())
            ? Rotation2d.kZero
            : input.getAngle();
        double curvedSpeed = Math.pow(input.getNorm(), SubsystemConstants.Swerve.kDriverPowFactor);

        Translation2d translation = new Translation2d(curvedSpeed, direction)
            .times(SubsystemConstants.Swerve.kDriverSpeedFactor)
            .times(maxSpeed);

        double curvedRotation = Math.pow(
            MathUtil.applyDeadband(Math.abs(rightX), deadband),
            SubsystemConstants.Swerve.kDriverRotationPowFactor
        );
        double rotation = -Math.signum(rightX)
            * curvedRotation
            * SubsystemConstants.Swerve.kDriverRotationSpeedFactor
            * maxAngularVelocity;

        SwerveController.get().setControl(new SwerveSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            SubsystemConstants.Swerve.kDriverFieldRelative
        ), "DRIVER");
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        SwerveController.get().periodic();
    }
}