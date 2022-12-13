package org.frogforce503.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

/**
 * This class is a thin wrapper around the CANSparkMax that reduces CAN bus /
 * CPU
 * overhead by skipping duplicate set commands. (By default the Spark flushes
 * the Tx buffer on every set call). Additionally, Low CAN utilization profiles
 * can be selected for when a device is idle.
 */

public class CANSparkMaxWrapper extends CANSparkMax {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = ControlMode.PercentOutput;
    protected CANProfile mLastCANProfile = CANProfile.Default;
    protected int mSlotID = 0;

    protected RelativeEncoder mEncoder;
    protected SparkMaxPIDController mPidController;

    public CANSparkMaxWrapper(int deviceNumber, MotorType motorType) {
        super(deviceNumber, motorType);

        mEncoder = this.getEncoder();
        mPidController = this.getPIDController();

        mPidController.setSmartMotionMinOutputVelocity(0, 0);
        // mPidController.setSmartMotionMinOutputVelocity(0, 1);
        // mPidController.setSmartMotionMinOutputVelocity(0, 2);

        mPidController.setOutputRange(-1, 1);
    }

    public ControlMode getControlMode() {
        return mLastControlMode;
    }

    public void set(ControlMode mode, double value, double arbFF) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;

            switch (mode) {
                case PercentOutput:
                    super.set(value);
                    break;
                case Voltage:
                    mPidController.setReference(value, ControlType.kVoltage, mSlotID);
                    break;
                case Velocity:
                    mPidController.setReference(value, ControlType.kVelocity, mSlotID, arbFF);
                    break;
                case SmartMotion:
                    mPidController.setReference(value, ControlType.kSmartMotion, mSlotID);
                    break;
                case Position:
                    mPidController.setReference(value, ControlType.kPosition, mSlotID);
                    break;
            }
        }
    }

    public void set(ControlMode mode, double value) {
        set(mode, value, 0.0);
    }

    public void setSensorPhase(boolean phase) {
        mEncoder.setPositionConversionFactor(phase ? 1 : -1);
        mEncoder.setVelocityConversionFactor(phase ? 1 : -1);
    }

    public void setP(int slotID, double kP) {
        mPidController.setP(kP, slotID);
    }

    public void setI(int slotID, double kI) {
        mPidController.setI(kI, slotID);
    }

    public void setD(int slotID, double kD) {
        mPidController.setD(kD, slotID);
    }

    public void setIzone(int slotID, double kIz) {
        mPidController.setIZone(kIz, slotID);
    }

    public void setFF(int slotID, double kFF) {
        mPidController.setFF(kFF, slotID);
    }

    public void setCruiseVelocity(int slotID, double velocity) {
        mPidController.setSmartMotionMaxVelocity(velocity, slotID);
    }

    public void setAcceleration(int slotID, double acceleration) {
        mPidController.setSmartMotionMaxAccel(acceleration, slotID);
    }

    public void setAllowableClosedLoopError(int slotID, double allowedErr) {
        mPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
    }

    public void selectProfileSlot(int slotID) {
        mSlotID = slotID;
    }

    public int getSelectedProfileSlot() {
        return mSlotID;
    }

    public double getEncoderPosition() {
        return mEncoder.getPosition();
    }

    public void setEncoderPosition(double position) {
        mEncoder.setPosition(position);
    }

    public void setPositionConversionFactor(double factor) {
        mEncoder.setPositionConversionFactor(factor);
    }

    public double getEncoderVelocity() {
        return mEncoder.getVelocity();
    }

    public double getClosedLoopError() {
        return mLastSet - getEncoderVelocity();
    }

    public enum ControlMode {
        SmartMotion, PercentOutput, Voltage, Velocity, Position
    }

    public void setCANProfile(CANProfile profile) {
        if (profile != mLastCANProfile) {
            mLastCANProfile = profile;
            switch (profile) {
                case Low: // Low Profiles for minimal CAN utilization
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250);
                    break;
                case Idle: // Idle Profile for CAN utilization(Call when leaving a motor in idle but may
                           // call again soon)
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250);
                    break;
                case Default: // Default Update Rates
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
                    super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
                    break;
            }
        }
    }

    public enum CANProfile {
        Low, Idle, Default
    }

}