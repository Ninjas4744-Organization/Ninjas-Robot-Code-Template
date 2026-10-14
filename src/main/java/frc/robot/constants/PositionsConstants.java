package frc.robot.constants;

import edu.wpi.first.networktables.DoubleSubscriber;
import frc.lib.NinjasLib.util.NinjasLogger;

public class PositionsConstants {
    public static class ExampleSubsystem {
        public static final DoubleSubscriber kShoot = NinjasLogger.tunable("Shooter/Shoot", 50.0);
        public static final DoubleSubscriber kShootFast = NinjasLogger.tunable("Shooter/Shoot Fast", 90.0);
    }
}
