package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverYReleasedCommand extends CommandBase {

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!StateEngine.getInstance().getDriverControls()) {
            Swerve.getInstance().snapToAngle(0);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}