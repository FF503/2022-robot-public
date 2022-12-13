package org.frogforce503.robot2022.tests.modes;

import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.TestStandModule;
import org.frogforce503.robot2022.tests.RobotTest;

public class SwerveSpin extends RobotTest {

    // double speed = n0
    double angle = 0;

    public SwerveSpin() {
        super();
        this.setName("swervespin");
    }

    @Override
    public void init() {
        Swerve.getInstance().setModuleRotation(0);
        angle = 0;
    }

    @Override
    public void run() {

        Swerve.getInstance().setModuleRotation(angle % 360);

        angle++;
        this.writeOutput("You are currrently running Swerve Spin Test");
    }

    @Override
    public void stop() {
        TestStandModule.getInstance().stop();
    }
}