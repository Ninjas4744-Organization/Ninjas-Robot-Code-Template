package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.subsystem.IO;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.ExampleSubsystem.ExampleState.*;

public class ExampleSubsystem extends StateMachineBase<ExampleSubsystem.ExampleState> implements
    ISubsystem.Resettable,
    ISubsystem.VelocityControlled,
    ISubsystem.GoalOriented<Double>,
    ISubsystem.Stoppable
{
    public enum ExampleState {
        UNKNOWN,
        RESET,
        IDLE,
        SHOOT
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    public ExampleSubsystem(boolean enabled) {
        super(ExampleState.class);
        currentState = UNKNOWN;

        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kExampleSubsystem);
            else
                this.io = new IO.All<>(){};
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("ExampleSubsystem", inputs);

        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.sequence(
            reset(),
            Commands.waitUntil(this::isReset)
        ));

        addEdge(RESET, IDLE);

        addEdge(IDLE, SHOOT, setVelocityCmd(PositionsConstants.ExampleSubsystem.kShoot.get()));
        addStateCommand(SHOOT, Commands.sequence(
            Commands.waitSeconds(1.5),
            setVelocityCmd(PositionsConstants.ExampleSubsystem.kShootFast.get())
        ));

        addEdge(SHOOT, IDLE, stopCmd());



        addStateEnd(RESET, () -> true, IDLE);

        addStateEnd(SHOOT, Seconds.of(3), IDLE);
    }

    @Override
    public void setVelocity(double velocity) {
        if (!enabled)
            return;

        io.setVelocity(velocity);
    }

    @Override
    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    @Override
    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
    }

    @Override
    public boolean atGoal(){
        if (!enabled)
            return true;

        return inputs.AtGoal;
    }

    @Override
    public Double getGoal() {
        if (!enabled)
            return 0.0;

        return inputs.Goal;
    }

    @Override
    public void stop() {
        if (!enabled)
            return;

        io.stopMotor();
    }

    @Override
    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(inputs.Velocity) < 5;
    }

    @Override
    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stopCmd();
    }
}