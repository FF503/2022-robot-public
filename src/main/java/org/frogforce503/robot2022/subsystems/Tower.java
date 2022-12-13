package org.frogforce503.robot2022.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.CANProfile;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.robot2022.Robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tower extends Subsystem {

    private double rpm;
    private double mTargetRpm = 0;
    private double towerDemand;
    private boolean velocityStabilized = true;

    private final double kTowerNeoKP = Robot.bot.kTowerNeoKP;
    private final double kTowerNeoKD = Robot.bot.kTowerNeoKD;
    private final double kTowerNeoKS = Robot.bot.kTowerNeoKS;
    private final double kTowerNeoKV = Robot.bot.kTowerNeoKV;

    private final double kTowerRevsPerMotor = Robot.bot.kTowerRevsPerMotor;

    private static Tower instance = null;
    private TowerStates curTowerState = TowerStates.OFF;

    private CANSparkMaxWrapper mTower;
    private DigitalInput towerBeam = new DigitalInput(Robot.bot.towerBeamID);
    Debouncer hasCargo = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
    Debouncer ballCounter = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    boolean lastHasCargo = false;
    boolean hasCargoPersistent = false;

    int BALL_SHOT_COUNT = 0;

    private final NetworkTable towerTable;
    private final ShuffleboardLayout towerTableList;

    public enum TowerStates {
        FEEDING, OFF, REVERSE, REVERSE_SLOW, SHOOT, MANUAL
    }

    private Tower() {
        towerTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Tower");

        towerTableList = Shuffleboard.getTab("GameSpec").getLayout("Tower", BuiltInLayouts.kList);

        mTower = new CANSparkMaxWrapper(Robot.bot.towerID, MotorType.kBrushless);
        mTower.restoreFactoryDefaults();
        mTower.setIdleMode(IdleMode.kBrake);
        mTower.setInverted(false);

        mTower.setSmartCurrentLimit(70); // effectively no current limiting happening

        mTower.setP(0, kTowerNeoKP);
        mTower.setD(0, kTowerNeoKD);
        mTower.setFF(0, kTowerNeoKV);
        mTower.setIzone(0, 0);

        towerDemand = 0;

        // Setup from shooter
        // mTower = new CANSparkMaxWrapper(Robot.bot.towerID, MotorType.kBrushless);
        // mTower.restoreFactoryDefaults();
        // mTower.setIdleMode(IdleMode.kBrake);
        // mTower.setInverted(false);
        // mTower.setSmartCurrentLimit(20);

        // mTower.setD(0, kTowerNeoKS);
        // mTower.setP(0, kTowerNeoKP);
        // mTower.setFF(0, kTowerNeoKV);
        // mTower.setIzone(0, 0);
        mTower.burnFlash();
    }

    public static Tower getInstance() {
        return instance == null ? instance = new Tower() : instance;
    }

    public boolean beamBroken() {
        return !towerBeam.get();
    }

    public boolean hasBall() {
        return hasCargo.calculate(beamBroken());
    }

    public void updateShotCount() {
        if (ballCounter.calculate(lastHasCargo == true && !hasBall())) {
            BALL_SHOT_COUNT++;
            lastHasCargo = false;
        }
        if (lastHasCargo == false) {
            lastHasCargo = hasBall();
        }
    }

    public int getShotCount() {
        return BALL_SHOT_COUNT;
    }

    public void clearHasBall() {
        hasCargoPersistent = false;
    }

    public TowerStates getTowerState() {
        return curTowerState;
    }

    public void setTowerState(TowerStates s) {
        curTowerState = s;
    }

    public void setTowerVelocity(double rpm) {
        mTargetRpm = rpm;
        towerDemand = (rpm / kTowerRevsPerMotor);
    }

    public boolean isVelocityStabilized() {
        return this.velocityStabilized;
    }

    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putString("Tower State", curTowerState.toString());
        velocityStabilized = Math.abs(mTargetRpm - getEncoderSpeedRpm()) < 50;
        rpm = getTowerSpeedRpm();
        if (curTowerState == TowerStates.OFF) {
            mTower.setCANProfile(CANProfile.Idle);
        } else {
            mTower.setCANProfile(CANProfile.Default);
        }
        switch (curTowerState) {
            case OFF:
                setTowerPower(0.0);
                break;
            case FEEDING:
                setTowerVelocity(Robot.bot.kTowerFeedingVel);
                setTowerDemand(towerDemand);
                break;
            case REVERSE:
                setTowerVelocity(-Robot.bot.kTowerShootingVel);
                setTowerDemand(towerDemand);
                break;
            case REVERSE_SLOW:
                setTowerVelocity(-Robot.bot.kTowerShootingVel * 0.5);
                setTowerDemand(towerDemand);
                break;
            case SHOOT:
                setTowerVelocity(Robot.bot.kTowerShootingVel);
                setTowerDemand(towerDemand);
                break;
            case MANUAL:
                setTowerDemand(towerDemand);
                break;
            default:
                curTowerState = TowerStates.OFF;
                break;
        }
        outputTelemetry();
    }

    private void setTowerPower(double power) {
        mTower.set(ControlMode.PercentOutput, power);
    }

    private void setTowerDemand(double target) {
        mTower.set(ControlMode.Velocity, target, kTowerNeoKS);
    }

    public void setTowerVelocityManual(double RPM) {
        setTowerState(TowerStates.MANUAL);
        towerDemand = RPM;
        setTowerVelocity(towerDemand);
    }

    public void initTelemetry() {
        towerTableList.withPosition(8, 0).withSize(2, 5);

        towerTableList.addBoolean("Tower Telemetry Toggle", () -> outputtingTelemetry())
                .withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);
        towerTableList.addBoolean("Tower Telemetry", () -> outputtingTelemetry())
                .withSize(2, 1);
        towerTableList.addString("Tower State", () -> curTowerState.toString())
                .withSize(1, 1);

        towerTableList.addNumber("Tower P", () -> towerTable.getEntry("Tower P").getDouble(kTowerNeoKP))
                .withSize(1, 1);
        towerTableList.addNumber("Tower D", () -> towerTable.getEntry("Tower D").getDouble(kTowerNeoKD))
                .withSize(1, 1);
        towerTableList.addNumber("Tower FF", () -> towerTable.getEntry("Tower FF").getDouble(kTowerNeoKV))
                .withSize(1, 1);
        towerTableList.addNumber("Tower RPM", () -> towerTable.getEntry("Tower RPM").getDouble(0.0))
                .withSize(1, 1);

    }

    public void outputTelemetry() {
        if (outputtingTelemetry()) {
            towerTable.getEntry("Tower Current").setNumber(mTower.getOutputCurrent());
            towerTable.getEntry("Tower Velocity Demand").setNumber(towerDemand);
            towerTable.getEntry("Tower Velocity").setNumber(getTowerSpeedRpm());
            towerTable.getEntry("Tower Beam Break RAW").setDouble(beamBroken() ? 1 : 0);
            towerTable.getEntry("Tower Beam Debounced").setDouble(hasBall() ? 1 : 0);
            towerTable.getEntry("Tower Ball COunt").setDouble(BALL_SHOT_COUNT);
        }
    }

    public void updatePIDValues() {
        mTower.setP(0, towerTable.getEntry("Tower P").getDouble(0.0));
        mTower.setD(0, towerTable.getEntry("Tower D").getDouble(0.0));
        mTower.setFF(0, towerTable.getEntry("Tower FF").getDouble(0.0));
        mTower.setIzone(0, 0);
    }

    public SparkMaxPIDController getPidValues() {
        return mTower.getPIDController();
    }

    private boolean outputtingTelemetry() {
        return towerTable.getEntry("Tower Telemetry Toggle").getBoolean(false);
    }

    @Override
    public void stop() {
        System.out.println("Stopping tower");
        setTowerState(TowerStates.OFF);
    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {
        writePeriodicOutputs();
    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }

    public double getSetpointRpm() {
        return towerDemand * kTowerRevsPerMotor;
    }

    private double getEncoderSpeedRpm() {
        return mTower.getEncoderVelocity();
    }

    private double getRollerSpeedRpm() {
        return mTower.getEncoderVelocity() * kTowerRevsPerMotor;
    }

    private double getTowerSpeedRpm() {
        return getEncoderSpeedRpm() * kTowerRevsPerMotor;
    }

    private double getIntakeSpeedError() {
        return rpm - getLastSetRpm();
    }

    private double getLastSetRpm() {
        return mTargetRpm;
    }

    // @Override
    // public void readPeriodicInputs() {
    // periodicIO.rpm = mIntake.getEncoderVelocity();
    // }

    // public class PeriodicIO {
    // // INPUTS
    // public double rpm;
    // public double encoderPosition; // used when we calculate our limits

    // // OUTPUTS
    // public double setpoint;
    // }
}
