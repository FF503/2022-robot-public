package org.frogforce503.robot2022.tests.modes;

import org.frogforce503.robot2022.subsystems.swerve.TestStandModule;
import org.frogforce503.robot2022.tests.RobotTest;

import edu.wpi.first.math.geometry.Rotation2d;

public class TestStandSpin extends RobotTest {

    // double speed = n0
    double angle = 0;

    public TestStandSpin() {
        super();
        this.setName("teststandspin");
    }

    @Override
    public void init() {
        TestStandModule.getInstance().setRotationPosition(Rotation2d.fromDegrees(0));
        angle = 0;
    }

    @Override
    public void run() {

        TestStandModule.getInstance().setRotationPosition(Rotation2d.fromDegrees(angle % 360));

        angle++;
        this.writeOutput("You are currrently running Test Stand Spin Test");
    }

    @Override
    public void stop() {
        TestStandModule.getInstance().stop();
    }
}