package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverBReleasedCommand extends CommandBase {
    @Override
    public void initialize() {
        // TODO Auto-generated method stub

    }

    @Override
    public void execute() {
        if (StateEngine.getInstance().getDriverControls()) {
            StateEngine.getInstance().setRobotState(RobotStates.IDLE);
            StateEngine.getInstance().disablePresetOverride();
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
