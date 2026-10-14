// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.NinjasLib.util.NinjasLogger;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    public Robot() {
        NinjasLogger.setOptions(new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogExtras(true));

        NinjasLogger.setEnabled(true);

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        robotContainer.controllerPeriodic();
        CommandScheduler.getInstance().run();
        robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.reset(true);
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        robotContainer.reset(false);

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        // stop all subsystems
        RobotContainer.getExampleSubsystem().stop();
        RobotContainer.getSwerveSubsystem().drive(0, 0, 0, 0);
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
