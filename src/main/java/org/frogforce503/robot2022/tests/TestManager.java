package org.frogforce503.robot2022.tests;

import java.time.LocalTime;
import java.util.HashMap;

import org.frogforce503.robot2022.subsystems.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class TestManager {
    private HashMap<String, RobotTest> tests = new HashMap<>();

    private String current;
    private String def;
    private static String log_string;

    SendableChooser<String> testChooser;

    private static NetworkTable testTable = null;

    public static void disable() {
        testTable.getEntry("enabled").setBoolean(false);
    }

    public static void log(String input) {
        log_string += input + "\n";
    }

    public static void clearLog() {
        log_string = "Cleared at " + LocalTime.now() + "\n";
    }

    public TestManager(RobotTest... tests) {
        this.def = tests[0].getName();
        this.testChooser = new SendableChooser<String>();
        this.current = this.def;

        for (RobotTest test : tests) {
            // if (test != null && test.getName() != null)
            this.tests.put(test.getName(), test);
            testChooser.addOption(test.getName(), test.getName());
        }

        Shuffleboard.getTab("TestMode").add(this.testChooser);
    }

    public void setActive(String name) {
        this.current = name;
    }

    public String checkActiveTest() {
        this.current = testChooser.getSelected().toString();
        return this.current;
    }

    public RobotTest getActive() {
        return this.tests.get(this.current);
    }

    public Subsystem getActiveSubsystem() {
        return this.tests.get(current).getSubsystem();
    }

    public String getDefault() {
        return this.def;
    }

    public RobotTest[] getTests() {
        Object[] values = this.tests.values().toArray();
        RobotTest[] testList = new RobotTest[values.length];

        int index = 0;
        for (Object value : values) {
            testList[index] = (RobotTest) value;
            index++;
        }

        return testList;
    }

    public String getLog() {
        return log_string;
    }

    public String[] getTestList() {
        Object[] keys = this.tests.keySet().toArray();
        String[] testList = new String[keys.length];

        int index = 0;
        for (Object key : keys) {
            testList[index] = key.toString();
            index++;
        }

        return testList;
    }

    public String getOutput() {
        return this.tests.get(current).getCurrentOutput();
    }

    public void handleInputTable(NetworkTable table) {
        if (testTable == null)
            testTable = table;

        this.tests.get(current).handleInputTable(table);
    }

    /**
     * SMARTDASHBOARD-ONLY FUNCTION
     */
    public void handleInput() {
        this.tests.get(current).handleInput();
    }

    public void run() {
        this.tests.get(current).run();
    }

    public void stop() {
        this.tests.get(current).stop();
    }

    public void init() {
        this.tests.get(current).init();
    }
}