package frc.robot.constants;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class PositionsConstants {
    public static class ExampleSubsystem {
        public final DoubleSubscriber kShoot = DogLog.tunable("Shooter/Shoot", 50.0);
        public final DoubleSubscriber kShootFast = DogLog.tunable("Shooter/Shoot Fast", 90.0);
    }
}
