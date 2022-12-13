package org.frogforce503.robot2022.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.CANProfile;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.RobotState;
import org.frogforce503.robot2022.StateEngine;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class JudgeZone extends Subsystem {

    private double rpm;
    private double mTargetRpm = 0;
    private double conveyorDemand;
    private boolean velocityStabilized = true;

    private final double kConveyorNeoKP = Robot.bot.kConveyorNeoKP;
    private final double kConveyorNeoKD = Robot.bot.kConveyorNeoKD;
    private final double kConveyorNeoKS = Robot.bot.kConveyorNeoKS;
    private final double kConveyorNeoKV = Robot.bot.kConveyorNeoKV;

    private final double kConveyorRevsPerMotor = Robot.bot.kConveyorRevsPerMotor;

    public double beamBreakTimeStamp = 0;
    private boolean ignorePixy = false;

    static JudgeZone instance;
    private ConveyorStates curConveyorState = ConveyorStates.OFF;
    private CANSparkMaxWrapper mConveyor;

    private final NetworkTable jzTable;
    private final ShuffleboardLayout jzTableList;

    Debouncer hasCargo = new Debouncer(0.125, DebounceType.kRising);

    public enum ConveyorStates {
        OFF, FEEDING, SHOOTING, EJECTING, EJECTING_SLOW, EJECTING_MEDIUM
    }

    public void setConveyorState(ConveyorStates s) {
        curConveyorState = s;
    }

    public ConveyorStates getConveyorState() {
        return curConveyorState;
    }

    public static JudgeZone getInstance() {
        return instance == null ? instance = new JudgeZone() : instance;
    }

    private DigitalInput judgeZoneSwitch = new DigitalInput(Robot.bot.conveyorBeamID);

    public enum BallColors {
        RED, BLUE, NONE
    }

    // private BallColors judgeZoneColor = BallColors.NONE;
    private BallColors towerColor = BallColors.NONE;
    private BallColors intakeColor = BallColors.NONE;

    private BallColors prevJzColor = BallColors.NONE;
    private BallColors curJzColor = BallColors.NONE;

    private boolean readyToFeed = false;

    public BallColors getTowerColor() {
        return towerColor;
    }

    public BallColors getIntakeColor() {
        return intakeColor;
    }

    public void setTowerColor(BallColors color) {
        towerColor = color;
    }

    public BallColors getJudgeZoneColor() {
        return curJzColor;
    }

    // public void setJudgeZoneColor(BallColors color) {
    // towerColor = color;
    // }

    private Pixy2 pixycam;
    boolean isCamera = false;
    int state = -1;

    // CANSparkMaxWrapper feedTowerMotor = new
    // CANSparkMaxWrapper(Robot.bot.conveyorID, MotorType.kBrushless);

    public JudgeZone() {
        jzTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("JudgeZone");

        jzTableList = Shuffleboard.getTab("GameSpec").getLayout("JudgeZone", BuiltInLayouts.kList);

        pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
        state = pixycam.init();
        isCamera = state >= 0;

        mConveyor = new CANSparkMaxWrapper(Robot.bot.conveyorID, MotorType.kBrushless);

        mConveyor.restoreFactoryDefaults();
        mConveyor.setIdleMode(IdleMode.kBrake);
        mConveyor.setInverted(true);
        mConveyor.setSmartCurrentLimit(20);

        mConveyor.setD(0, kConveyorNeoKD);
        mConveyor.setP(0, kConveyorNeoKP);
        mConveyor.setFF(0, kConveyorNeoKV);
        mConveyor.setIzone(0, 0);

        conveyorDemand = 0;
        mConveyor.burnFlash();
    }

    // Sees if camera has connection or not
    public boolean isCamera() {
        return isCamera;
    }

    private ArrayList<Block> blocks = new ArrayList<Block>();

    // Updates Image Seen
    public void updateImage() {
        pixycam.getCCC().getBlocks(false, 255, 255); // run getBlocks with arguments to have the camera acquire target
                                                     // data
        blocks = pixycam.getCCC().getBlockCache();
    }

    // Return Color of Image Identified
    public BallColors getColor() {
        if (blocks.size() > 0) {
            return (blocks.get(0).getSignature() == 1) ? BallColors.BLUE : BallColors.RED;
        }
        return BallColors.NONE;
    }

    public boolean hasBall() {
        return !judgeZoneSwitch.get();
    }

    public void updateBallColors() {
        setJudgeZoneColor(getColor());
        setTowerColor(prevJzColor);
    }

    public boolean isBallColorMatching() {
        if (ignorePixy) {
            return true;
        } else {
            updateBallColors();
            return (curJzColor == RobotState.getInstance().getAllianceColor());
        }
    }

    @Override
    public void writePeriodicOutputs() {
        updateImage();
        if (curConveyorState == ConveyorStates.OFF) {
            mConveyor.setCANProfile(CANProfile.Idle);
        } else {
            mConveyor.setCANProfile(CANProfile.Default);
        }
        switch (curConveyorState) {
            case OFF:
                stopFeeder();
                mConveyor.setCANProfile(CANProfile.Idle);
                break;

            case FEEDING:
                setVelocity(Robot.bot.kConveyorIntakingVelocity);
                setConveyorDemand(conveyorDemand);
                break;

            case SHOOTING:
                setVelocity(Robot.bot.kConveyorVelocity);
                setConveyorDemand(conveyorDemand);
                break;

            case EJECTING:
                setVelocity(-Robot.bot.kConveyorVelocity);
                setConveyorDemand(conveyorDemand);
                break;

            case EJECTING_SLOW:
                setVelocity(-Robot.bot.kConveyorVelocity * 0.5);
                setConveyorDemand(conveyorDemand);
                break;

            case EJECTING_MEDIUM:
                setVelocity(-Robot.bot.kConveyorVelocity * 0.6);
                setConveyorDemand(conveyorDemand);
                break;

            default:
                setConveyorState(ConveyorStates.OFF);
                break;
        }
    }

    private void setJudgeZoneColor(BallColors s) {
        if (s != curJzColor) {
            prevJzColor = curJzColor;
        }
        curJzColor = s;
    }

    public void togglePixyCam() {
        if (ignorePixy) {
            ignorePixy = false;
        } else {
            ignorePixy = true;
        }
    }

    public boolean isReadyToFeed() {
        return readyToFeed;
    }

    public void runFeeder(double percentSpeed) {
        mConveyor.set(ControlMode.PercentOutput, percentSpeed);
    }

    private void setConveyorDemand(double target) {
        mConveyor.set(ControlMode.Velocity, target, kConveyorNeoKS);
    }

    public void stopFeeder() {
        mConveyor.set(ControlMode.PercentOutput, 0.0);
    }

    public void initTelemetry() {
        jzTableList.withPosition(2, 0).withSize(2, 5);

        jzTableList.addBoolean("JudgeZone Telemetry Toggle", () -> outputtingTelemetry())
                .withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);
        jzTableList.addBoolean("JudgeZone Telemetry", () -> outputtingTelemetry())
                .withSize(2, 1);
        // jzTableList.addString("JudgeZone State", () -> curConveyorState.toString())
        // .withSize(1, 1);
        jzTableList.addNumber("JZ P", () -> jzTable.getEntry("JZ P").getDouble(kConveyorNeoKP))
                .withSize(1, 1);
        jzTableList.addNumber("JZ D", () -> jzTable.getEntry("JZ D").getDouble(kConveyorNeoKD))
                .withSize(1, 1);
        jzTableList.addNumber("JZ FF", () -> jzTable.getEntry("JZ FF").getDouble(kConveyorNeoKV))
                .withSize(1, 1);
    }

    public void outputTelemetry() {
        if (outputtingTelemetry()) {
            jzTable.getEntry("Conveyor Current").setNumber(mConveyor.getOutputCurrent());
            // jzTable.getEntry("Conveyor Power").setNumber(mConveyor.get());
            jzTable.getEntry("Conveyor Velocity Demand").setNumber(conveyorDemand);
            jzTable.getEntry("Conveyor Velocity").setNumber(getConveyorSpeedRpm());
            jzTable.getEntry("has Cargo RAW").setDouble(hasBall() ? 1 : 0);

            jzTable.getEntry("has Cargo").setBoolean(hasBall());
            jzTable.getEntry("PixyCam Color Detected").setString(getColor().toString());
            jzTable.getEntry("PixyCam RED").setDouble(getColor() == BallColors.RED ? 1 : 0);
            jzTable.getEntry("PixyCam BLUE").setDouble(getColor() == BallColors.BLUE ? 1 : 0);
            jzTable.getEntry("PixyCam NONE").setDouble(getColor() == BallColors.NONE ? 1 : 0);
            jzTable.getEntry("PixyCam ISCOLORMATCHING").setDouble(isBallColorMatching() ? 1 : 0);
            jzTable.getEntry("PixyCam HASBALL").setDouble(JudgeZone.getInstance().hasBall() ? 1 : 0);

        }
    }

    public void updatePIDValues() {
        mConveyor.setP(0, jzTable.getEntry("JZ P").getDouble(0.0));
        mConveyor.setD(0, jzTable.getEntry("JZ D").getDouble(0.0));
        mConveyor.setFF(0, jzTable.getEntry("JZ FF").getDouble(0.0));
        mConveyor.setIzone(0, 0);
    }

    private boolean outputtingTelemetry() {
        return jzTable.getEntry("JudgeZone Telemetry Toggle").getBoolean(false);
    }

    // Subsystem Overriden Methods (Not Used)

    @Override
    public void stop() {
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
    }

    @Override
    public void onStop(double timestamp) {
    }

    public void setVelocity(double rpm) {
        mTargetRpm = rpm;
        conveyorDemand = (rpm / kConveyorRevsPerMotor);
    }

    public boolean isVelocityStabilized() {
        return this.velocityStabilized;
    }

    public double getSetpointRpm() {
        return conveyorDemand * kConveyorRevsPerMotor;
    }

    private double getEncoderSpeedRpm() {
        return mConveyor.getEncoderVelocity();
    }

    private double getConveyorSpeedRpm() {
        return getEncoderSpeedRpm() * kConveyorRevsPerMotor;
    }

    private double getConveyorSpeedError() {
        return rpm - getLastSetRpm();
    }

    private double getLastSetRpm() {
        return mTargetRpm;
    }

    public void verifyBallCount() {
        // if (JudgeZone.getInstance().jzCargo() && Shooter.getInstance().hasCargo()) {
        // StateEngine.getInstance().setBallCount(2);
        // } else if (JudgeZone.getInstance().jzCargo() ^
        // Shooter.getInstance().hasCargo()) {
        // StateEngine.getInstance().setBallCount(1);
        // } else {
        // StateEngine.getInstance().setBallCount(0);
        // }

        if (getColor() != BallColors.NONE /* Shooter.getInstance().hasCargo() */) {
            StateEngine.getInstance().setBallCount(1);
        } else {
            StateEngine.getInstance().setBallCount(0);
        }
    }
}
