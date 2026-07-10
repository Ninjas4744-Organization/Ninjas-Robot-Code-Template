package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.ExampleSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private LoggedCommandController driverController;
    private LoggedCommandController operatorController;

    private static ExampleSubsystem exampleSubsystem;

    public RobotContainer() {
        if (!GeneralConstants.kRobotMode.isReplay()) {
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
//            operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIOPS5(GeneralConstants.kOperatorControllerPort));
        } else {
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
//            operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIO() {});
        }

        RobotStateBase.set(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));

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
        Logger.recordOutput("Output1", 5);

        if(GeneralConstants.kRobotMode.isSim())
            SimulatedArena.getInstance().simulationPeriodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
