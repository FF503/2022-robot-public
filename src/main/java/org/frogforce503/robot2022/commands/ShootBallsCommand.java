package org.frogforce503.robot2022.commands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.Tower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBallsCommand extends CommandBase {

    double startShootingTimestamp;
    int startShotCount;
    int numWanted;
    boolean finished;

    double timeout;

    public ShootBallsCommand() {
        this(2);
    }

    public ShootBallsCommand(int numWanted) {
        this(numWanted, (0.75 * numWanted));
    }

    public ShootBallsCommand(int numWanted, double timeout) {
        this.numWanted = numWanted;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        startShootingTimestamp = Timer.getFPGATimestamp();
        startShotCount = Tower.getInstance().getShotCount();
        StateEngine.getInstance().setRobotState(RobotStates.AUTON_PRE_SHOOT);
    }

    @Override
    public void execute() {
        boolean minTimeout = Timer.getFPGATimestamp() - startShootingTimestamp > 1.0;
        boolean maxTimeout = Timer.getFPGATimestamp() - startShootingTimestamp > timeout;
        boolean shotsDone = Tower.getInstance().getShotCount() - startShotCount >= numWanted;
        // boolean robotEmpty = (!JudgeZone.getInstance().hasBall() &&
        // !Tower.getInstance().hasBall());

        // System.out.println("CHECKING CONDITION " + shotsDone + " shot count " +
        // Tower.getInstance().getShotCount());
        finished = (minTimeout && shotsDone) || maxTimeout;
        // System.out.println("Mintimeout: " + minTimeout);
        // System.out.println("Maxtimeout: " + maxTimeout);
        // System.out.println("Shotsdone: " + shotsDone);

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
