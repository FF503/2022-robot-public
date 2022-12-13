package org.frogforce503.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands. (By default the Talon flushes
 * the Tx buffer on every set call).
 */
public class TalonSRXWrapper extends TalonSRX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    protected CANProfile mLastCANProfile = CANProfile.Default;

    public TalonSRXWrapper(int deviceNumber) {
        super(deviceNumber);
        super.configFactoryDefault();
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }

    public void setCANProfile(CANProfile profile) {
        if (profile != mLastCANProfile) {
            mLastCANProfile = profile;
            switch (profile) {
                case Low: // Low Profiles for minimal CAN utilization
                    super.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
                    super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 200);
                    super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 200);
                    break;
                case Idle: // Idle Profile for CAN utilization(Call when leaving a motor in idle but may
                           // call again soon)
                    super.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
                    super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
                    super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100);
                    break;
                case Default: // Default Update Rates
                    super.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
                    super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
                    super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100);
                    break;
            }
        }
    }

    public enum CANProfile {
        Low, Idle, Default
    }
}