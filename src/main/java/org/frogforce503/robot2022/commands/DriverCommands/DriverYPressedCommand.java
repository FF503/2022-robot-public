package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverYPressedCommand extends CommandBase {

    final double WALL_SHOT = 0; // FIXME

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!StateEngine.getInstance().getDriverControls()) {
            StateEngine.getInstance().setRobotState(RobotStates.REVERSE_TOWER);
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