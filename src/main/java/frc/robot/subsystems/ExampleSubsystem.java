package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;

import java.util.List;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.ExampleSubsystem.ExampleState.*;

public class ExampleSubsystem extends StateMachineBase<ExampleSubsystem.ExampleState> {
    public enum ExampleState {
        UNKNOWN,
        RESET,
        IDLE,
        SHOOT,
        SHOOT_FAST
    }

    private Controller controller;
    private final boolean enabled;

    public ExampleSubsystem(boolean enabled) {
        super(ExampleState.class);

        this.enabled = enabled;
        if (enabled) {
            controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kExampleSubsystem);
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        controller.periodic();
        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.sequence(
            stopCmd(),
            Commands.waitUntil(() -> controller.getOutput() < 0.1)
        ));

        addEdge(RESET, IDLE);

        addEdge(List.of(IDLE, SHOOT_FAST), SHOOT, () -> Commands.sequence(
            setVelocityCmd(PositionsConstants.ExampleSubsystem.kShoot.get()),
            waitUntilGoal()
        ));

        addEdge(List.of(IDLE, SHOOT), SHOOT_FAST, () -> Commands.sequence(
            setVelocityCmd(PositionsConstants.ExampleSubsystem.kShootFast.get()),
            waitUntilGoal()
        ));

        addStateCommand(SHOOT_FAST, Commands.run(() -> {
            System.out.println("Shooting fast: " + getVelocity());
        }));

        addStateEnd(SHOOT_FAST, Seconds.of(3), IDLE);
    }

    public boolean atGoal() {
        if (enabled)
            return controller.atVelocityGoal();
        return true;
    }

    public Command waitUntilGoal() {
        return Commands.waitUntil(() -> atGoal());
    }

    public void setPercent(double percent) {
        if (enabled)
            controller.setPercent(percent);
    }

    public Command setPercentCmd(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    public void setVelocity(double velocity) {
        if (enabled)
            controller.setVelocity(velocity);
    }

    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public double getVelocity() {
        if (enabled)
            return controller.getVelocity();
        return 0;
    }

    public void stop() {
        if (enabled)
            controller.stop();
    }

    public Command stopCmd() {
        return Commands.runOnce(() -> stop());
    }
}