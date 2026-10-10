package frc.robot.subsystems;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.SubsystemConstants;

public class ExampleSubsystem extends StateMachineBase<ExampleSubsystem.ExampleState>
{
    public enum ExampleState {
        UNKNOWN,
        RESET,
        IDLE,
        SHOOT
    }

    private Controller motor;

    public ExampleSubsystem(boolean enabled) {
        super(ExampleState.class);
        if (enabled){
            motor = Controller.createController(
                Controller.ControllerType.TalonFX,
                SubsystemConstants.kExampleSubsystem
            );
        }
    }

    public void setPercent(double percent)  { motor.setPercent(percent); }
    public void setHeight(double meters)    { motor.setPosition(meters); }
    public void stop()                      { motor.stop(); }
    public boolean atGoal()                 { return motor.atGoal(); }

    @Override
    public void periodic() {
        motor.periodic();
    }

    @Override
    protected void define() {

    }
}