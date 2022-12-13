package org.frogforce503.robot2022.commands;

import org.frogforce503.robot2022.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForIntakeCurrentCommand extends CommandBase {

    int startBallCount = 0;
    int wantedBalls = 0;
    double startTime = 0;

    public WaitForIntakeCurrentCommand(int wantedBalls) {
        this(wantedBalls, 0);
    }

    public WaitForIntakeCurrentCommand(int wantedBalls, int startBallCount) {
        this.startBallCount = startBallCount;
    }

    @Override
    public void initialize() {
        if (startBallCount == 0)
            startBallCount = Intake.getInstance().getBallCount();

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return (Intake.getInstance().getBallCount() - startBallCount >= wantedBalls)
                || Timer.getFPGATimestamp() - startTime > 0.75;
    }
}
