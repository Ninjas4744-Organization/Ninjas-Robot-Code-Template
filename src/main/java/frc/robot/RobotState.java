package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;

public class RobotState extends RobotStateBase {
    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
    }

    public static RobotState get() {
        return (RobotState) RobotStateBase.get();
    }
}
