package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.robot.constants.GeneralConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {
    private final LoggedCommandController driverController;
    private final LoggedCommandController operatorController;

    private static SwerveSubsystem swerveSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static ExampleSubsystem exampleSubsystem;

    public RobotContainer() {
        driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
        operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIOPS5(GeneralConstants.kOperatorControllerPort));

        swerveSubsystem = new SwerveSubsystem(true);
        visionSubsystem = new VisionSubsystem(true);
        exampleSubsystem = new ExampleSubsystem(false);

        configureBindings();
    }

    public static SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }
    public static VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
    public static ExampleSubsystem getExampleSubsystem() {
        return exampleSubsystem;
    }

    private void configureBindings() {
        // Driver bindings
        driverController.cross().onTrue(notTest(exampleSubsystem.changeStateCommand(ExampleSubsystem.ExampleState.SHOOT)));
        driverController.circle().onTrue(notTest(exampleSubsystem.changeStateCommand(ExampleSubsystem.ExampleState.IDLE)));

        // Operator bindings

        // Test bindings
        driverController.cross().whileTrue(inTest(Commands.startEnd(
            () -> exampleSubsystem.setPercent(0.2),
            () -> exampleSubsystem.stop()
        )));

        driverController.circle().whileTrue(inTest(Commands.startEnd(
            () -> exampleSubsystem.setPercent(-0.2),
            () -> exampleSubsystem.stop()
        )));
    }

    private Command inTest(Command command) {
        return Commands.either(
            command,
            Commands.none(),
            () -> !DriverStation.isTest()
        );
    }

    private Command notTest(Command command) {
        return Commands.either(
            command,
            Commands.none(),
            () -> DriverStation.isTest()
        );
    }

    // Only for updating controllers
    public void controllerPeriodic() {
        driverController.periodic();
        operatorController.periodic();
    }

    public void periodic() {
        NinjasLogger.log("Output1", 5);

        swerveSubsystem.drive(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX(), driverController.getRightY());

        if (Robot.isSimulation())
            SimulatedArena.getInstance().simulationPeriodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void reset(boolean autonomous) {
        if (autonomous) {
            SwerveController.get().setChannel("AUTO");
        } else {
            SwerveController.get().setChannel("DRIVER");
        }

        swerveSubsystem.drive(0, 0, 0, 0);

        exampleSubsystem.forceState(ExampleSubsystem.ExampleState.UNKNOWN);
        exampleSubsystem.changeState(ExampleSubsystem.ExampleState.IDLE);
    }
}
