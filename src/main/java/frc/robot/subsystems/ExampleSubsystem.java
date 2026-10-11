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
    private boolean enabled = false;

    public ExampleSubsystem(boolean enabled) {
        super(ExampleState.class);
        if (enabled){
            enabled = true;
            motor = Controller.createController(
                Controller.ControllerType.TalonFX,
                SubsystemConstants.kExampleSubsystem
            );
        }
    }

    public void setPercent(double percent)  {
        if (enabled)
            motor.setPercent(percent);
    }
    public void setHeight(double meters)    {
        if (enabled)
            motor.setPosition(meters);
    }
    public void stop()                      {
        if (enabled)
            motor.stop();
    }
    public boolean atGoal(){
        if (enabled)
            return motor.atGoal();
        return true;
    }

    @Override
    public void periodic() {
        if (enabled)
            motor.periodic();
    }

    @Override
    protected void define() {

    }
}