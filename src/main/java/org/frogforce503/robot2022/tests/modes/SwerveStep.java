package org.frogforce503.robot2022.tests.modes;

import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.tests.RobotTest;

public class SwerveStep extends RobotTest {

    // double speed = n0
    double angle = 0;

    public SwerveStep() {
        super();
        this.setName("swervestep");
    }

    @Override
    public void init() {
        Swerve.getInstance().setModuleRotation(0);
        angle = 0;
    }

    @Override
    public void run() {

        Swerve.getInstance().setModuleRotation(angle % 360);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        angle += 90;
        this.writeOutput("You are currrently running Swerve Spin Test");
    }

    @Override
    public void stop() {
        Swerve.getInstance().stop();
    }
}