package org.frogforce503.robot2022.subsystems.swerve;

import org.frogforce503.robot2022.Robot;

public class TestStandModule extends SwerveModule {

    private static TestStandModule instance = null;

    public TestStandModule() {
        super(Robot.bot.backLeftName, Swerve.ModuleLocation.TestStandModule);
    }

    public static TestStandModule getInstance() {
        if (instance == null)
            instance = new TestStandModule();

        return instance;
    }

    // public void writePeriodicOutputs() {

    // }
}
