package org.frogforce503.robot2022.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.CANProfile;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    private double rpm;
    private double mTargetRpm = 0;
    private double intakeDemand;
    private boolean velocityStabilized = true;

    private final double kIntakeNeoKP = Robot.bot.kIntakeNeoKP;
    private final double kIntakeNeoKD = Robot.bot.kIntakeNeoKD;
    private final double kIntakeNeoKS = Robot.bot.kIntakeNeoKS;
    private final double kIntakeNeoKV = Robot.bot.kIntakeNeoKV;

    private final double kIntakeRevsPerMotor = Robot.bot.kIntakeRevsPerMotor;

    private static Intake instance = null;
    private IntakeStates curIntakeState = IntakeStates.OFF;

    private CANSparkMaxWrapper mIntake;

    private DoubleSolenoid intakeShifter;

    // public PeriodicIO periodicIO = new PeriodicIO();
    private double lastCurrentSpikeTime = 0;
    private boolean intakeJammed = false;

    private final NetworkTable intakeTable;
    private final ShuffleboardLayout intakeTableList;

    private String current;
    /// Ball count data
    double lastDifference = 0;
    double lastAccel = 0;
    double lastRpm = 0;
    double lastTime = Timer.getFPGATimestamp();
    double acceleration = 0;
    Value lastShifterPos = null;
    private int INTAKE_BALL_COUNT = 0;
    ///

    public enum IntakeStates {
        INTAKING, AUTON_INTAKING, OFF, DOWN, REVERSE, EJECTING
    }

    private Intake() {
        intakeTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Intake");
        intakeTableList = Shuffleboard.getTab("GameSpec").getLayout("Intake", BuiltInLayouts.kList);

        mIntake = new CANSparkMaxWrapper(Robot.bot.intakeID, MotorType.kBrushless);
        mIntake.restoreFactoryDefaults();
        mIntake.setIdleMode(IdleMode.kCoast);
        mIntake.setInverted(true);
        mIntake.setP(0, kIntakeNeoKP);
        mIntake.setD(0, kIntakeNeoKD);
        mIntake.setFF(0, kIntakeNeoKV);
        mIntake.setIzone(0, 0);
        mIntake.setSmartCurrentLimit(Robot.bot.kIntakeCurrentLimit, Robot.bot.kIntakeCurrentLimit + 10);

        mIntake.burnFlash();

        intakeShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Robot.bot.shiftForwardID,
                Robot.bot.shiftReverseID);

        intakeDemand = 0;

    }

    public static Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }

    public IntakeStates getIntakeState() {
        return curIntakeState;
    }

    public boolean intakeStateIsTrue() {
        return getIntakeState() == IntakeStates.INTAKING;
    }

    public void setIntakeState(IntakeStates s) {
        curIntakeState = s;
    }

    public void setRollerVelocity(double rpm) {
        mTargetRpm = rpm;
        intakeDemand = (rpm / kIntakeRevsPerMotor);
    }

    public void adjustIntakeVelocity(double rpm) {
        double target = rpm
                + Math.abs((Swerve.getInstance().getVelocity().getX() * Constants.kIntakeRatio));
        setRollerVelocity(target);
    }

    public boolean isVelocityStabilized() {
        return this.velocityStabilized;
    }

    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putString("Intake State", curIntakeState.toString());
        if (!intakeJammed && mIntake.getOutputCurrent() >= 50
                && (Timer.getFPGATimestamp() - lastCurrentSpikeTime) > 1) {
            lastCurrentSpikeTime = Timer.getFPGATimestamp();
            // intakeJammed = true;
        }

        if (curIntakeState == IntakeStates.OFF) {
            mIntake.setCANProfile(CANProfile.Idle);
        } else {
            mIntake.setCANProfile(CANProfile.Default);
        }

        velocityStabilized = Math.abs(mTargetRpm - getEncoderSpeedRpm()) < 50;
        rpm = getIntakeSpeedRpm();
        switch (curIntakeState) {
            case OFF:
                setIntakePower(0.0);
                setShifter(Value.kReverse);
                break;
            case DOWN:
                setIntakePower(0.0);
                setShifter(Value.kForward);
                break;
            case INTAKING:
                adjustIntakeVelocity(Robot.bot.kIntakeVelocity); // FIXME:
                setIntakeDemand(intakeDemand);
                setShifter(Value.kForward);
                break;
            case AUTON_INTAKING:
                setRollerVelocity(Robot.bot.kAutoIntakeVelocity);
                setIntakeDemand(intakeDemand);
                setShifter(Value.kForward);
                break;
            case REVERSE:
                setRollerVelocity(-Robot.bot.kIntakeVelocity);
                setIntakeDemand(intakeDemand);
                setShifter(Value.kForward);
                if (intakeJammed && (Timer.getFPGATimestamp() - 0.5) > lastCurrentSpikeTime) {
                    // intakeJammed = false;
                    break;
                }

                break;
            case EJECTING:
                setShifter(Value.kReverse);
                setIntakePower(0.0);

                break;
            default:
                curIntakeState = IntakeStates.OFF;
                break;
        }
        outputTelemetry();
    }

    private void setIntakePower(double power) {
        mIntake.set(ControlMode.PercentOutput, power);
    }

    private void setIntakeDemand(double target) {
        mIntake.set(ControlMode.Velocity, target, kIntakeNeoKS);
    }

    private void setShifter(Value value) {
        if (value != lastShifterPos) {
            intakeShifter.set(value);
            lastShifterPos = value;
        }
    }

    public int getBallCount() {
        return this.INTAKE_BALL_COUNT;
    }

    public void resetShotCount() {
        this.INTAKE_BALL_COUNT = 0;
    }

    public void updateBallCount() {

        double rpm = getIntakeSpeedRpm();
        double error = intakeDemand - rpm;
        double dt = Timer.getFPGATimestamp() - lastTime;
        double acc = (rpm - lastRpm) / dt;

        double percentage = (rpm / intakeDemand);

        // Checking if we have reached a relative minimum on our acceleration
        boolean min = Math.signum(acc) == 1 && (Math.signum(lastAccel) == -1 || Math.signum(lastAccel) == 0);

        if ((curIntakeState == IntakeStates.INTAKING || curIntakeState == IntakeStates.AUTON_INTAKING)
                && ((percentage < 0.85) && (percentage > 0.65)) && min) {
            INTAKE_BALL_COUNT++;
        }

        lastDifference = error;
        lastAccel = acc;
        lastRpm = rpm;
        lastTime = Timer.getFPGATimestamp();
        acceleration = acc;
    }

    public void initTelemetry() {
        intakeTableList.withPosition(4, 0).withSize(2, 5);

        intakeTableList.addBoolean("Intake Telemetry Toggle",
                () -> outputtingTelemetry()).withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);
        intakeTableList.addBoolean("Intake Telemetry",
                () -> outputtingTelemetry()).withSize(2, 1);
        intakeTableList.addString("Intake State",
                () -> curIntakeState.toString()).withSize(1, 1);

        intakeTableList.addNumber("Intake P", () -> intakeTable.getEntry("Intake P").getDouble(kIntakeNeoKP)).withSize(
                1,
                1);
        intakeTableList.addNumber("Intake D", () -> intakeTable.getEntry("Intake D").getDouble(kIntakeNeoKD))
                .withSize(1, 1);
        intakeTableList.addNumber("Intake FF", () -> intakeTable.getEntry("Intake FF").getDouble(kIntakeNeoKV))
                .withSize(1, 1);

        // Shuffleboard.getTab("DriverStation").addNumber("Battery Voltage",
        // mIntake.getBusVoltage());
    }

    public void outputTelemetry() {
        if (outputtingTelemetry()) {
            intakeTable.getEntry("Intake Current").setNumber(mIntake.getOutputCurrent());
            intakeTable.getEntry("Intake Ball Counter").setNumber(getBallCount());
            intakeTable.getEntry("Intake Acceleration").setNumber(acceleration);
            intakeTable.getEntry("Intake Velocity Demand").setNumber(intakeDemand);
            intakeTable.getEntry("Intake Velocity").setNumber(getRollerSpeedRpm());
            // intakeTable.getEntry("Intake Power").setNumber(mIntake.get());
            intakeTable.getEntry("Intake Shifter Position")
                    .setString(intakeShifter.get() == Value.kForward ? "OUT" : "IN");
        }
    }

    public void updatePIDValues() {
        mIntake.setP(0, intakeTable.getEntry("Intake P").getDouble(0.0));
        mIntake.setD(0, intakeTable.getEntry("Intake D").getDouble(0.0));
        mIntake.setFF(0, intakeTable.getEntry("Intake FF").getDouble(0.0));
        mIntake.setIzone(0, 0);
    }

    private boolean outputtingTelemetry() {
        return intakeTable.getEntry("Intake Telemetry Toggle").getBoolean(false);
    }

    @Override
    public void stop() {
        System.out.println("Stopping intake");
        setIntakeState(IntakeStates.OFF);
    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {
        // writePeriodicOutputs();
    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }

    public double getSetpointRpm() {
        return intakeDemand * kIntakeRevsPerMotor;
    }

    private double getEncoderSpeedRpm() {
        return mIntake.getEncoderVelocity();
    }

    private double getRollerSpeedRpm() {
        return mIntake.getEncoderVelocity() * Robot.bot.kIntakeRevsPerMotor;
    }

    private double getIntakeSpeedRpm() {
        return getEncoderSpeedRpm() * kIntakeRevsPerMotor;
    }

    private double getIntakeSpeedError() {
        return rpm - getLastSetRpm();
    }

    private double getLastSetRpm() {
        return mTargetRpm;
    }

    public boolean isIntakeJammed() {
        return intakeJammed;
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
