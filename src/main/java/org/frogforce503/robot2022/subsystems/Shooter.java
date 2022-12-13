package org.frogforce503.robot2022.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import org.frogforce503.lib.drivers.TalonFXWrapper;
import org.frogforce503.lib.util.InterpolatingDouble;
import org.frogforce503.lib.util.InterpolatingTreeMap;
import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.vision.LimelightProcessor;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {

    TalonFXWrapper mShooter;
    private double targetSpeed;
    // FIXME: set target speed this to requested speed from turret

    private static Shooter instance = null;
    // private BallColors towerBallColor = BallColors.NONE;
    private final PeriodicIO periodicIO = new PeriodicIO();

    private boolean velocityStabilized = false;
    private final double kShooterFalconKP = Robot.bot.kShooterFalconKP;
    private final double kShooterFalconKI = Robot.bot.kShooterFalconKI;
    private final double kShooterFalconKD = Robot.bot.kShooterFalconKD;
    private final double kShooterFalconKS = Robot.bot.kShooterFalconKS;
    private final double kShooterFalconKFF = Robot.bot.kSHooterFalconkFF;

    private static final int kTimeoutMs = 100;
    private final double kShooterRevsPerMotor = Robot.bot.kShooterRevsPerMotor;

    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToShooterSpeed = new InterpolatingTreeMap<>();

    private final double kTowerNeoKP = Robot.bot.kTowerNeoKP;
    private final double kTowerNeoKD = Robot.bot.kTowerNeoKD;
    private final double kTowerNeoKV = Robot.bot.kTowerNeoKV;
    private final double kTowerNeoKS = Robot.bot.kTowerNeoKS;
    private final double kTowerRevsPerMotor = Robot.bot.kTowerRevsPerMotor;

    private final double SHOOTER_MIN_VELOCITY_TOLERANCE = -30;
    private final double SHOOTER_MAX_VELOCITY_TOLERANCE = 70;
    private final double SHOOTER_VELOCITY_TOLERANCE = 30;

    private final double SHOOTER_SHOT_ACCEL = 10;
    private double lastDifference = 0;
    private double lastAccel = 0;
    private int BALL_SHOT_COUNT = 0;
    private double SHOOTING_ADJUST = 0;

    private double mTargetRpm = 0;
    private LinearFilter rpmFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public final String kShooterRPMoffset = "Shooter RPM Offset";

    public enum ShooterStates {
        IDLE, EJECT, OPEN_LOOP, AUTON_VISION, VISION, SET_DISTANCE, MANUAL, OFF
    }

    public enum TowerSwitchStates {
        BROKEN, COMPLETE, NULL
    }

    private ShooterStates curShooterState = ShooterStates.OFF;

    public ShooterStates getShooterState() {
        return curShooterState;
    }

    public void setShooterState(ShooterStates s) {
        curShooterState = s;
    }

    public static Shooter getInstance() {

        return instance = instance == null ? new Shooter() : instance;
    }

    private final NetworkTable shooterTable, shotmapShooterTable, shotmapOutTable;
    private final ShuffleboardLayout shooterTableList, shotmapShooterList, shotmapOutList;

    public double getCurrentSensorSpeed() {
        return mShooter.getSelectedSensorVelocity();
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public int getShotCount() {
        return this.BALL_SHOT_COUNT;
    }

    public void resetShotCount() {
        this.BALL_SHOT_COUNT = 0;
    }

    public boolean isShooterReady() {
        checkVelocityStabilized();
        return this.velocityStabilized;
    }

    // public BallColors getTowerBallColor() {
    // return JudgeZone.getInstance().getColor();
    // }

    // public void setTowerBallColor(BallColors towerBallColor) {
    // this.towerBallColor = towerBallColor;
    // }

    public void populateTreeMapForShooter() {
        for (int i = 0; i < Robot.bot.shotMapValues.length; i++) {
            kDistanceToShooterSpeed.put(new InterpolatingDouble(Robot.bot.shotMapValues[i][0]),
                    new InterpolatingDouble(Robot.bot.shotMapValues[i][1]));
        }
    }

    public Shooter() {

        populateTreeMapForShooter();

        shooterTableList = Shuffleboard.getTab("GameSpec").getLayout("Shooter", BuiltInLayouts.kList);
        shooterTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Shooter");

        shotmapShooterList = Shuffleboard.getTab("Shotmap").getLayout("Shooter", BuiltInLayouts.kList);
        shotmapShooterTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Shotmap")
                .getSubTable("Shooter");

        shotmapShooterTable.getEntry("Shotmap Shooter RPM Offset")
                .setDouble(Preferences.getDouble(kShooterRPMoffset, 0.0));

        shotmapOutList = Shuffleboard.getTab("Shotmap").getLayout("mapOut", BuiltInLayouts.kList);
        shotmapOutTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Shotmap")
                .getSubTable("mapOut");

        mShooter = new TalonFXWrapper(Robot.bot.shooterID);

        mShooter.configFactoryDefault();
        mShooter.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, Constants.kShooterCurrentLimit,
                Constants.kShooterCurrentLimit + 5, 1.0));
        mShooter.selectProfileSlot(0, 0);
        mShooter.config_kP(0, kShooterFalconKP, kTimeoutMs);
        mShooter.config_kI(0, kShooterFalconKI, kTimeoutMs);
        mShooter.config_IntegralZone(0, RPMToEncoderCounts(600));
        mShooter.config_kD(0, kShooterFalconKD, kTimeoutMs);
        mShooter.config_kF(0, kShooterFalconKFF, kTimeoutMs);
        mShooter.setNeutralMode(NeutralMode.Coast);
        mShooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.rpm = getShooterSpeedRpm();
        periodicIO.dt = Timer.getFPGATimestamp() - periodicIO.lastTime;
        periodicIO.acceleration = (periodicIO.rpm - periodicIO.lastRpm) / periodicIO.dt;
        periodicIO.lastRpm = periodicIO.rpm;
        periodicIO.lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putString("Shooter State", curShooterState.toString());
        periodicIO.rpm = getShooterSpeedRpm();
        double targetVelocity;
        if (shotmapTuningActive()) {
            setShooterVelocityManual(shotmapRPMsetpoint());
        }
        switch (curShooterState) {
            case OPEN_LOOP:
                mShooter.set(TalonFXControlMode.PercentOutput, periodicIO.shooterDemand);
                break;

            case SET_DISTANCE:
                targetVelocity = getShooterSpeedForDistance(167.0) / Robot.bot.kShooterRevsPerMotor;

                periodicIO.shooterDemand = targetVelocity;

                setShooterVelocity(targetVelocity);
                break;
            case VISION:
                targetVelocity = SHOOTING_ADJUST
                        + getShooterSpeedForDistance(LimelightProcessor.getInstance().getDistFromHub())
                                / Robot.bot.kShooterRevsPerMotor;

                if (LimelightProcessor.getInstance().getTV() == 0.0) {
                    // target velocity = last target velocity
                }
                if (StateEngine.getInstance().getRobotState() != RobotStates.SHOOTING) {
                    periodicIO.shooterDemand = targetVelocity;
                }
                setShooterVelocity((periodicIO.shooterDemand + shotmapRPMoffset()));
                break;
            case AUTON_VISION:
                targetVelocity = SHOOTING_ADJUST
                        + getShooterSpeedForDistance(LimelightProcessor.getInstance().getDistFromHub())
                                / Robot.bot.kShooterRevsPerMotor;

                // if (LimelightProcessor.getInstance().getTV() == 0.0) {
                // // target velocity = last target velocity
                // }

                if (StateEngine.getInstance().getRobotState() != RobotStates.AUTON_SHOOTING) {
                    periodicIO.shooterDemand = targetVelocity;
                }
                // periodicIO.shooterDemand = targetVelocity;

                setShooterVelocity(targetVelocity + shotmapRPMoffset());
                break;
            case MANUAL:
                targetVelocity = periodicIO.shooterDemand;
                setShooterVelocity(targetVelocity);
                break;
            case OFF:
                setShooterPower(0);
                break;
            case IDLE:
                if (periodicIO.rpm > Robot.bot.kShooterIdle + 50) {
                    setShooterPower(0.0);
                } else {
                    periodicIO.shooterDemand = Robot.bot.kShooterIdle;
                    setShooterVelocity(periodicIO.shooterDemand);
                }
            default:
                setShooterState(ShooterStates.IDLE);
                break;
        }
    }

    private void setShooterPower(double power) {
        mShooter.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setShooterVelocityManual(double RPM) {
        periodicIO.shooterDemand = RPM;
        setShooterState(ShooterStates.MANUAL);
    }

    public void adjustAutonShootingVelocity(double extraRPM) {
        SHOOTING_ADJUST = extraRPM;
    }

    public void setBatterShot() {
        setShooterVelocityManual(Robot.bot.kShooterBatterRPM);
    }

    public void setPresetShot(double presetRPM) {
        setShooterVelocityManual(presetRPM);
    }

    private void setShooterVelocity(double shooterRPM) {
        double motorRPM = shooterRPM / Robot.bot.kShooterRevsPerMotor;
        motorRPM *= (Constants.falconClicksperRevolution / 600); // converts RPM to clicks / 100ms
        mShooter.set(TalonFXControlMode.Velocity, motorRPM);

        // checkVelocityStabilized();
    }

    public double getShooterSpeedForDistance(double distance) {
        double speed = kDistanceToShooterSpeed
                .getInterpolated(new InterpolatingDouble(
                        Math.max(Math.min(distance, Constants.kMaxShootingDist), Constants.kMinShootingDist))).value;

        SmartDashboard.putNumber("CALCULATED SHOOTER SPEED", speed);
        SmartDashboard.putNumber("CALCULATED SHOOTER DISTANCE", distance);

        // System.out.println("GETTING SHOOTER SPEED FOR DISTANCE " + distance + " SPEED
        // " + speed);

        return speed;
    }

    public void updateShotCount() {
        // https://stats.stackexchange.com/questions/41145/simple-way-to-algorithmically-identify-a-spike-in-recorded-errors
        // periodicIO.movingAvg = rpmFilter.calculate(periodicIO.rpm);
        double rpm = getShooterSpeedRpm();
        double error = periodicIO.shooterDemand - rpm;
        double dt = Timer.getFPGATimestamp() - periodicIO.lastTime;
        double acceleration = (rpm - periodicIO.lastRpm) / dt;

        boolean min = Math.signum(acceleration) == 1 && (Math.signum(lastAccel) == -1 || Math.signum(lastAccel) == 0);

        if (curShooterState != ShooterStates.IDLE && periodicIO.shooterDemand != Robot.bot.kShooterIdle
                && ((rpm / periodicIO.shooterDemand) < 0.9) && min) {
            BALL_SHOT_COUNT++;
        }
        SmartDashboard.putNumber("Shooter Motor Accel", acceleration);

        lastDifference = error;
        lastAccel = acceleration;
        periodicIO.lastRpm = rpm;
        periodicIO.lastTime = Timer.getFPGATimestamp();
        periodicIO.acceleration = acceleration;

        // SmartDashboard.putNumber("Shot Count", value);
        // SmartDashboard.putNumber("Moving Avg Dip Amount",
        // ((periodicIO.rpm - periodicIO.movingAvg) / periodicIO.movingAvg));
    }

    public double getSetpointRpm() {
        return periodicIO.shooterDemand * kShooterRevsPerMotor;
    }

    // ENCODER COUNTS Per100MS
    private double RPMToEncoderCounts(double RPM) {
        return RPM * (Constants.falconClicksperRevolution / 600.0);
    }

    // Return value in RPM instead of counts/100ms
    private double getEncoderSpeedRpm() {
        return mShooter.getSelectedSensorVelocity() / (Constants.falconClicksperRevolution / 600.0);
    }

    private double getShooterSpeedRpm() {
        return getEncoderSpeedRpm() * kShooterRevsPerMotor;
    }

    private double getShooterSpeedError() {
        return periodicIO.rpm - getLastSetRpm();
    }

    private double getLastSetRpm() {
        return mTargetRpm;
    }

    private void checkVelocityStabilized() {
        // System.out.println("VELOCITY stabalized" + Math.abs(periodicIO.shooterDemand
        // - getEncoderSpeedRpm()));
        // System.out.println("ENCRPM" + getEncoderSpeedRpm());
        velocityStabilized = Math.abs(
                (periodicIO.shooterDemand + shotmapRPMoffset()) - getEncoderSpeedRpm()) < SHOOTER_VELOCITY_TOLERANCE;

        // velocityStabilized = (periodicIO.shooterDemand - getEncoderSpeedRpm() <
        // SHOOTER_MAX_VELOCITY_TOLERANCE)
        // && (periodicIO.shooterDemand - getEncoderSpeedRpm() >
        // SHOOTER_MIN_VELOCITY_TOLERANCE);
    }

    public class PeriodicIO {
        // Inputs
        public double rpm;
        public double lastRpm;

        // Acceleration
        public double dt;
        public double acceleration; // Units of rot / (min * sec)
        public double lastTime = 0;

        // Outputs
        public double shooterDemand;
        public double movingAvg;
    }

    public void initTelemetry() {
        // Shooter
        shooterTableList.withPosition(0, 0).withSize(2, 5);

        shooterTableList.addBoolean("Shooter Telemetry Toggle", () -> outputtingTelemetry())
                .withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);

        shooterTableList.addBoolean("Shooter Telemetry", () -> outputtingTelemetry())
                .withSize(2, 1);

        shooterTableList.addString("Shooter State", () -> curShooterState.toString())
                .withSize(1, 1);
        // Shotmap
        shotmapShooterList.withPosition(0, 0).withSize(3, 5);

        shotmapShooterList
                .addNumber("Shotmap Shooter RPM Setpoint", () -> (Math.round(shotmapRPMsetpoint() / 10.0) * 10))
                .withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 500, "max", 5000));
        shotmapShooterList.addNumber("Shotmap Shooter RPM Number", () -> getShooterSpeedRpm())
                .withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 500, "max", 5000));

        shotmapShooterList
                .addNumber("Shotmap Shooter RPM Offset", () -> (Math.round(shotmapRPMoffset() / 10.0) * 10))
                .withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("Subtract", -200, "Add", 200));

        shotmapOutList.withPosition(3, 0).withSize(3, 5);
        shotmapOutList.addBoolean("Shotmap Tuning Toggle", () -> shotmapTuningActive())
                .withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);
        shotmapOutList.addBoolean("Store Entry", () -> toggleStoreShotmapEntry())
                .withSize(1, 1).withWidget(BuiltInWidgets.kToggleSwitch);
        shotmapOutList.addBoolean("Shotmap Tuning Active", () -> shotmapTuningActive())
                .withSize(2, 1);
        shotmapOutList.addBoolean("Shotmap Output", () -> toggleStoreShotmapEntry())
                .withSize(1, 1);

        shotmapOutList
                .addString("Shotmap Output String",
                        () -> shotmapOutTable.getEntry("Shotmap Output").getString(tempShotMapString))
                .withSize(1, 3);

        shooterTableList.addNumber("Shooter P", () -> shooterTable.getEntry("Shooter P").getDouble(kShooterFalconKP))
                .withSize(1, 1);
        shooterTableList.addNumber("Shooter I", () -> shooterTable.getEntry("Shooter I").getDouble(kShooterFalconKI))
                .withSize(1, 1);
        shooterTableList.addNumber("Shooter D", () -> shooterTable.getEntry("Shooter D").getDouble(kShooterFalconKD))
                .withSize(1, 1);
        shooterTableList.addNumber("Shooter FF", () -> shooterTable.getEntry("Shooter FF").getDouble(kShooterFalconKFF))
                .withSize(1, 1);

        Shuffleboard.getTab("DriverStation").addNumber("BUS VOLTAGE", mShooter::getBusVoltage);
    }

    public void updatePIDValues() {
        mShooter.config_kP(0, shooterTable.getEntry("Shooter P").getDouble(0.0), kTimeoutMs);
        mShooter.config_kI(0, shooterTable.getEntry("Shooter I").getDouble(0.0), kTimeoutMs);
        mShooter.config_kD(0, shooterTable.getEntry("Shooter D").getDouble(0.0), kTimeoutMs);
        mShooter.config_kF(0, shooterTable.getEntry("Shooter FF").getDouble(0.0), kTimeoutMs);
    }

    String tempShotMapString = "";

    public void outputTelemetry() {
        if (outputtingTelemetry()) {
            shooterTable.getEntry("Shooter Current").setNumber(mShooter.getStatorCurrent());
            shooterTable.getEntry("Shooter Voltage").setNumber(mShooter.getMotorOutputVoltage());
            shooterTable.getEntry("Shooter RPM Setpoint").setNumber(periodicIO.shooterDemand);
            shooterTable.getEntry("Shooter RPM").setNumber(getShooterSpeedRpm());
            shooterTable.getEntry("Shooter Proportional Error").setNumber(periodicIO.rpm / periodicIO.shooterDemand);
            shooterTable.getEntry("IZONE APPLLIED INPUT").setNumber(mShooter.getIntegralAccumulator());
            shooterTable.getEntry("Shooter Acceleration").setNumber(periodicIO.acceleration);
            shooterTable.getEntry("Shooter READY").setBoolean(isShooterReady());
            shooterTable.getEntry("Shooter Ball Shot Count").setNumber(BALL_SHOT_COUNT);
        }

        if (shotmapTuningActive()) {
            shotmapShooterTable.getEntry("Shooter RPM Setpoint").setNumber(periodicIO.shooterDemand);
            // shotmapShooterTable.getEntry("Shooter RPM").setNumber(getShooterSpeedRpm());
            shotmapShooterTable.getEntry("Distance To Target")
                    .setNumber(LimelightProcessor.getInstance().getDistFromHub());
            // shooterTable.getEntry("Shotmap Shooter RPM Setpoint").setNumber(0.0);
        } else {
            tempShotMapString = ""; // clear shotmap string once done tuning shotmap?
        }

        if (toggleStoreShotmapEntry()) {
            String shotmapString = shotmapOutTable.getEntry("Shotmap Output String").getString(tempShotMapString);

            tempShotMapString = shotmapString + ", {" +
                    (int) LimelightProcessor.getInstance().getDistFromHub()
                    + " , " + (int) periodicIO.shooterDemand +
                    " , " + (int) Hood.getInstance().getHoodSetpoint() + "}";

            shotmapOutTable.getEntry("Store Entry").setBoolean(false);
        }

        shotmapOutTable.getEntry("Shotmap Output String").setString(tempShotMapString);

    }

    private boolean outputtingTelemetry() {
        return shooterTable.getEntry("Shooter Telemetry Toggle").getBoolean(false);
    }

    private double shotmapRPMsetpoint() {
        return shotmapShooterTable.getEntry("Shotmap Shooter RPM Setpoint").getDouble(0.0);
    }

    private double shotmapRPMoffset() {
        return shotmapShooterTable.getEntry("Shotmap Shooter RPM Offset")
                .getDouble(0.0);
    }

    private boolean shotmapTuningActive() {
        return shotmapOutTable.getEntry("Shotmap Tuning Toggle").getBoolean(false);
    }

    private boolean toggleStoreShotmapEntry() {
        return shotmapOutTable.getEntry("Store Entry").getBoolean(false);
    }

    public void updatePreferences() {
        Preferences.setDouble(kShooterRPMoffset,
                shotmapShooterTable.getEntry("Shotmap Shooter RPM Offset").getDouble(0.0));

        // Preferences.setDouble(kShooterRPMoffset,
        // shotmapShooterTable.getEntry("Shotmap Shooter RPM
        // Offset").getDouble(defaultValue);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        setShooterState(ShooterStates.OFF);
    }

    @Override
    public void onStart(double timestamp) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onLoop(double timestamp) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onStop(double timestamp) {
        // TODO Auto-generated method stub

    }
}
