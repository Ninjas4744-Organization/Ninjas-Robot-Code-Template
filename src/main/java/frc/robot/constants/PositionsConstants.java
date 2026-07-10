package frc.robot.constants;

import frc.lib.NinjasLib.LoggedTunableNumber;

public class PositionsConstants {
    public static class ExampleSubsystem {
        public static final LoggedTunableNumber kShoot = new LoggedTunableNumber("Shooter/Shoot", 50, false);
        public static final LoggedTunableNumber kShootFast = new LoggedTunableNumber("Shooter/Shoot Fast", 90, false);
    }
}
