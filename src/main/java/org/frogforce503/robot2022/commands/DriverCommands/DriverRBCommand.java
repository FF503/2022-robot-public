package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverRBCommand extends CommandBase {

    final double SAFE_ZONE_DISTANCE = 0; // FIXME

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (StateEngine.getInstance().getClimbingSwap()) {
            Climber.getInstance().toggleRotate();
        } else {
            StateEngine.getInstance().togglePersistentOverride();
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
