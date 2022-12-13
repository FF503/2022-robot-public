package org.frogforce503.robot2022.tests.modes;

import org.frogforce503.robot2022.subsystems.Turret;
import org.frogforce503.robot2022.tests.RobotTest;

public class TurretReZero extends RobotTest {

    int direction = 1;
    boolean justPressed = false;

    int leftEncoderCount = 0;
    int rightEncoderCount = 0;

    public TurretReZero() {
        super();
        this.setName("turretrezero");
    }

    @Override
    public void init() {
        Turret.getInstance().setOpenLoop(0.0);
    }

    @Override
    public void run() {
        if ((Turret.getInstance().getCcwLimitPressed() || Turret.getInstance().getCwLimitPressed()) && !justPressed) {
            System.out.println("");
            if (direction == 1) {
                direction *= -1;
                rightEncoderCount = Turret.getInstance().turretAngleToEncUnits(Turret.getInstance().getAngle());
            } else {
                direction = 0;
                leftEncoderCount = Turret.getInstance().turretAngleToEncUnits(Turret.getInstance().getAngle());
            }

            justPressed = true;
        } else {
            justPressed = false;
        }

        Turret.getInstance().setOpenLoop(direction * 0.3);
        Turret.getInstance().writePeriodicOutputs();

        this.writeOutput("Right Encoder Count: " + rightEncoderCount + " Left Encoder Count: " + leftEncoderCount
                + " Average: " + ((rightEncoderCount + leftEncoderCount) / 2));
    }

    @Override
    public void stop() {
        Turret.getInstance().setOpenLoop(0.0);
    }
}