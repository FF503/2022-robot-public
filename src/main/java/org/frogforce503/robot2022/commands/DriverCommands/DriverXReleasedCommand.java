package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverXReleasedCommand extends CommandBase {
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (StateEngine.getInstance().getDriverControls()) {
            // do nothing
        } else {
            StateEngine.getInstance().setToPreviousState();
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
