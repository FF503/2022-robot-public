package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverBPressedCommand extends CommandBase {
    @Override
    public void initialize() {
        // TODO Auto-generated method stub

    }

    @Override
    public void execute() {
        if (StateEngine.getInstance().getDriverControls()) {
            StateEngine.getInstance().enablePresetOverride();
            if (StateEngine.getInstance().getRobotState() != RobotStates.SHOOTING) {
                StateEngine.getInstance().setRobotState(RobotStates.PRE_SHOOT);
            }
        } else {
            Swerve.getInstance().setAngle(0);
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
