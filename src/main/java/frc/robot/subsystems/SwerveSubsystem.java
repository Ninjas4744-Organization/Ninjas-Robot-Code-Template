package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.SubsystemConstants;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem() {
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
    }

    @Override
    public void periodic() {
        Swerve.get().periodic(); // must be called every loop
    }
}