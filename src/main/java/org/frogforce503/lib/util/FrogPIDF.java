package org.frogforce503.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class FrogPIDF {
    private final double p, i, d, f;
    private final ControlMode control;
    private double state;
    private double tolerance;
    private double setpoint;
    private double lastTime, lastError = 0;
    private double integral;
    private double error;

    public FrogPIDF(double p, double i, double d, ControlMode controlMode) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.control = controlMode;
        this.f = 0.0;
    }

    public FrogPIDF(double p, double i, double d, double f, ControlMode controlMode) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.control = controlMode;
        this.f = (control == ControlMode.Velocity_Control) ? f : 0.0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        this.integral = 0;
        this.lastTime = Timer.getFPGATimestamp();
    }

    public void updateSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void initializePIDF() {
        this.integral = 0;
        this.lastTime = Timer.getFPGATimestamp();
    }

    public double calculateOutput(double sensorState, boolean boundTo180) {
        this.state = sensorState;
        double error = boundTo180 ? Util.boundAngleNeg180to180Degrees(setpoint - sensorState)
                : (setpoint - sensorState);

        this.error = error;
        if (Math.abs(error) < tolerance && !boundTo180) {
            return 0.0;
        }
        double dError = error - lastError;
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        double derivative = dError / dt;
        integral += error * dt;
        double pOut = p * error;
        double iOut = i * integral;
        double dOut = d * derivative;
        double fOut = f * setpoint;

        lastTime = time;
        lastError = error;

        return Math.max(-1, Math.min(pOut + iOut + dOut + fOut, 1));
    }

    public void setTolerance(double t) {
        this.tolerance = t;
    }

    public boolean onTarget() {
        return Math.abs(state - setpoint) < tolerance;
    }

    public double getError() {
        return error;
    }

    public enum ControlMode {
        Velocity_Control, Position_Control
    }

}