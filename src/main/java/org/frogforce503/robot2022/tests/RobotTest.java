package org.frogforce503.robot2022.tests;

import org.frogforce503.robot2022.subsystems.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class RobotTest {
    private Subsystem subsystem;
    private String name;
    private String currentOutput = "---";

    @SuppressWarnings("unchecked")
    private TestInput<Object>[] inputs = new TestInput[] {};

    public abstract void run();

    public abstract void stop();

    public void init() {

    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Subsystem setSubsystem(Subsystem subsystem) {
        return this.subsystem = subsystem;
    }

    public Subsystem getSubsystem() {
        return this.subsystem;
    }

    public void writeOutput(String output) {
        this.currentOutput = output;
    }

    public void setInputs(TestInput<Object>[] inputs) {
        this.inputs = inputs;
    }

    public TestInput<Object>[] getInputs() {
        return this.inputs;
    }

    public void handleInput() {
        for (TestInput<Object> input : this.inputs) {
            switch (input.getType()) {
            case "String":
                input.setValue(SmartDashboard.getString(input.name, input.getValue().toString()));
                break;
            case "Double":
                input.setValue(SmartDashboard.getNumber(input.name, Double.valueOf(input.getValue().toString())));
                break;
            default:
                input.setValue(SmartDashboard.getBoolean(input.name, Boolean.valueOf(input.getValue().toString())));
            }
        }
    }

    public void handleInputTable(NetworkTable table) {
        for (TestInput<Object> input : this.inputs) {
            NetworkTableEntry entry = table.getEntry("i" + input.name);
            switch (input.getType()) {
            case "String":
                input.setValue(entry.getString(input.getValue().toString()));
                break;
            case "Double":
                input.setValue(entry.getDouble(Double.parseDouble(input.getValue().toString())));
                break;
            default:
                input.setValue(entry.getBoolean(Boolean.parseBoolean(input.getValue().toString())));
            }
        }
    }

    public String getCurrentOutput() {
        return this.currentOutput;
    }
}