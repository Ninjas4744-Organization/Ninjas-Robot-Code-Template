package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.constants.GeneralConstants;
import frc.robot.subsystems.ExampleSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private LoggedCommandController driverController;
    private LoggedCommandController operatorController;

    private static ExampleSubsystem exampleSubsystem;

    public RobotContainer() {
        driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
        operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIOPS5(GeneralConstants.kOperatorControllerPort));

        exampleSubsystem = new ExampleSubsystem(true);

        configureBindings();
    }

    public static ExampleSubsystem getExampleSubsystem() {
        return exampleSubsystem;
    }

    private void configureBindings() {
        driverController.cross().onTrue(exampleSubsystem.changeStateCommand(ExampleSubsystem.ExampleState.SHOOT));
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        SwerveSpeeds driverInput = new SwerveSpeeds(
                driverController.getLeftY(),
                driverController.getLeftY(),
                driverController.getRightX(),
                true
        );

        SwerveController.get().setControl(driverInput, "DRIVER");
        Logger.recordOutput("Output1", 5);

        if(GeneralConstants.kRobotMode.isSim())
            SimulatedArena.getInstance().simulationPeriodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
