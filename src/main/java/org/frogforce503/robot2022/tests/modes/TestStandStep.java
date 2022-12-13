package org.frogforce503.robot2022.tests.modes;

import org.frogforce503.robot2022.subsystems.swerve.TestStandModule;
import org.frogforce503.robot2022.tests.RobotTest;

import edu.wpi.first.math.geometry.Rotation2d;

public class TestStandStep extends RobotTest {

    // double speed = n0
    double angle = 0;
    boolean direction = false;

    public TestStandStep() {
        super();
        // this.setSubsystem(TestStandModule.getInstance());
        this.setName("teststandstep");
    }

    @Override
    public void init() {
        TestStandModule.getInstance().setRotationPosition(Rotation2d.fromDegrees(0));
        angle = 0;
    }

    @Override
    public void run() {
        // TestStandModule.getInstance().setRotationPosition(Rotation2d.fromDegrees(angle
        // % 360));
        TestStandModule.getInstance().setRotationPosition(Rotation2d.fromDegrees(angle % 360));

        angle += 90;

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.writeOutput("You are currrently running the Test Stand Step Test");
    }

    @Override
    public void stop() {
        TestStandModule.getInstance().stop();
    }
}