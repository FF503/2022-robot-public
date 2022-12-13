package org.frogforce503.robot2022.commands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForIntakeBallsCommand extends CommandBase {

    boolean finished = false;
    double timeout = 2.5;
    double startTimestamp = 0;

    public WaitForIntakeBallsCommand() {
        this(2.5);
    }

    public WaitForIntakeBallsCommand(double timeout) {
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        startTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (startTimestamp == 0)
            startTimestamp = Timer.getFPGATimestamp();

        if (Timer.getFPGATimestamp() - startTimestamp >= timeout)
            finished = true;
        else
            finished = StateEngine.getInstance().getRobotState() == RobotStates.IDLE;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
