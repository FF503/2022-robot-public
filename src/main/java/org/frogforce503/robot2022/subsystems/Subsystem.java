package org.frogforce503.robot2022.subsystems;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot
 * subsystems. Each subsystem outputs commands to SmartDashboard, has a stop
 * routine (for after each match), and a routine to zero all sensors, which
 * helps with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two
 * drivetrains), and functions get the instance of the drivetrain and act
 * accordingly. Subsystems are also a state machine with a desired state and
 * actual state; the robot code will try to match the two states with actions.
 * Each Subsystem also is responsible for instantializing all member components
 * at the start of the match.
 */
public abstract class Subsystem {
    public boolean hasEmergency = false;

    public void writeToLog() {
    }

    // call during periodic functions in Robot class
    public void readPeriodicInputs() {
    }

    // call during periodic functions in Robot class
    public void writePeriodicOutputs() {
    }

    public abstract void outputTelemetry();

    public abstract void stop();

    public abstract void onStart(double timestamp);

    public abstract void onLoop(double timestamp);

    public abstract void onStop(double timestamp);

    public void setCurrentLimit(int limit) {
    }

    // public void registerEnabledLoops(ILooper enabledLooper) {
    // }

    public void zeroSensors() {
    }
}