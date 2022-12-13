package org.frogforce503.robot2022.tests.modes;

import java.util.HashMap;

import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.tests.RobotTest;

public class ReZeroSwerveRoutine extends RobotTest {

    String rezeroOutput = "ReZero Routine Output: ";
    boolean hasMeasured = false;

    public ReZeroSwerveRoutine() {
        super();
        this.setName("swerve_rezero_routine");
    }

    @Override
    public void init() {

    }

    @Override
    public void run() {
        if (!hasMeasured) {
            HashMap<String, Double> rezeroValues = Swerve.getInstance().getModuleRezeroValues();
            for (String moduleName : rezeroValues.keySet()) {
                rezeroOutput += moduleName + ": " + (-rezeroValues.get(moduleName)) + ", ";
            }
            rezeroOutput.substring(0, rezeroOutput.length() - 2);
            hasMeasured = true;
        }
        this.writeOutput(rezeroOutput);
    }

    @Override
    public void stop() {

    }
}
