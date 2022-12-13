package org.frogforce503.robot2022.tests;

import org.frogforce503.robot2022.subsystems.Subsystem;

public class RobotTestBuilder extends RobotTest {
    private Runnable func;
    private Runnable onStop;

    public RobotTestBuilder(Subsystem subsystem, String name, Runnable func) {
        super();
        this.setSubsystem(subsystem);
        this.setName(name);
        this.func = func;
    }

    public RobotTestBuilder(Subsystem subsystem, String name, Runnable func, Runnable onStop) {
        super();
        this.setSubsystem(subsystem);
        this.setName(name);
        this.func = func;
        this.onStop = onStop;
    }

    @Override
    public void run() {
        this.func.run();
    }

    @Override
    public void stop() {
        if (this.onStop != null)
            this.onStop.run();
    }
}