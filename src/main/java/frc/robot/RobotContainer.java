package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.localization.RobotPose;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.lib.NinjasLib.util.NinjasAutoBuilder;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
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

    private SendableChooser<String> autoChooser;

    public RobotContainer() {
        driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
        operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIOPS5(GeneralConstants.kOperatorControllerPort));

        swerveSubsystem = new SwerveSubsystem(true);
        visionSubsystem = new VisionSubsystem(true);
        exampleSubsystem = new ExampleSubsystem(true);

        configureAuto();
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

    private void configureAuto() {
        NinjasAutoBuilder.configure(
            RobotPose.get()::getRobotPose,
            RobotPose.get()::setRobotPose,
            Swerve.get()::getSpeeds,
            (speeds, feedforwards) -> SwerveController.get().setControl(new SwerveSpeeds(speeds, false), "AUTO"),
            SubsystemConstants.Swerve.kAutonomyConfig,
            SubsystemConstants.Swerve.kRobotConfig,
            () -> false
        );

        NamedCommands.registerCommand("SHOOT", exampleSubsystem.changeStateCommand(ExampleSubsystem.ExampleState.SHOOT));
        NamedCommands.registerCommand("STOP", exampleSubsystem.changeStateCommand(ExampleSubsystem.ExampleState.IDLE));

        autoChooser = NinjasAutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("Mirror Auto", false);
    }

    public String getAutonomousCommandName() {
        return autoChooser.getSelected();
    }

    public PathPlannerAuto getAutonomousCommand() {
        return NinjasAutoBuilder.buildAuto(getAutonomousCommandName(), SmartDashboard.getBoolean("Mirror Auto", false));
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
