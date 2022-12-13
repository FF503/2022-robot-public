package org.frogforce503.robot2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import org.frogforce503.lib.drivers.TalonSRXWrapper;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.vision.LimelightProcessor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends Subsystem {

    private static double maxHardLimit;
    private static double minHardLimit;

    private static double kTurretEncRange;

    private static double limelightCenterOffset = 6.5; // inches
    private static double fixedTx = 0.0;

    private static double kEncZeroOffset;
    private static double kEncBatterOffset;
    private static double kEncHomeOffset;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetDeg = 0.0;
    private final double HUB_X = 8;
    private final double HUB_Y = 4;
    private double curTargetX = HUB_X;
    private double curTargetY = HUB_Y;
    private double targetDemand = 0.0;

    private double kTurretLockingThreshold = 75.0;

    private double turretEncoderZero;
    private double turretEncoderBatter;
    private double turretEncoderHome;
    private double turretEncoderBack;

    private static Turret instance = null;
    TalonSRXWrapper mTurretMotor;
    DigitalInput mCwLimit, mCcwLimit;
    PeriodicIO periodicIO = new PeriodicIO();
    // private boolean overrideSkew = true;
    private double targetAngle = 0.0;
    private TurretStates currentState = TurretStates.DISABLED;
    private HomePositions homePosition = HomePositions.RIGHT;
    private double kTranslationFeedforward = 0.0; // FIXME
    private double kRotationFeedForward = 0.0; // FIXME

    Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final NetworkTable turretTable;
    private final ShuffleboardLayout turretTableList;

    public enum TurretStates {
        BATTER, PERCENT_ROTATION, VISION_LOCKED, AUTON_LOCKED, VISION_MOVING, POSITION, FIELD_FORWARD,
        AUTO_HOME, DISABLED, TURRET_HOME, FIELD_RELATIVE, TURRET_BACK, SHOOTING_PRESET
    }

    public enum HomePositions {
        RIGHT, BATTER, BACK
    }

    private Turret() {

        if (Robot.bot.turretHasGadgeteerLimits) {
            maxHardLimit = Robot.bot.kTurretMaxCCWEncLimit;
            minHardLimit = Robot.bot.kTurretMaxCWEncLimit;
        } else {
            maxHardLimit = Robot.bot.kTurretMaxCCWEncLimit;
            minHardLimit = Robot.bot.kTurretMaxCWEncLimit;
        }

        kTurretEncRange = maxHardLimit - minHardLimit;

        kEncZeroOffset = Robot.bot.kTurretEncoderZeroOffset;
        kEncBatterOffset = Robot.bot.kTurretEncoderBatterOffset;
        kEncHomeOffset = Robot.bot.kTurretEncoderHomeOffset;

        turretEncoderZero = minHardLimit + kEncZeroOffset;
        turretEncoderBatter = minHardLimit + kEncBatterOffset;
        turretEncoderHome = minHardLimit + kEncHomeOffset;
        turretEncoderBack = turretEncoderBatter + (4096 / 2);

        turretTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Turret");
        turretTableList = Shuffleboard.getTab("GameSpec").getLayout("Turret", BuiltInLayouts.kList);

        mTurretMotor = new TalonSRXWrapper(Robot.bot.turretID);

        // Limit switches will vary depending on how setup
        // mTurretMotor.configFactoryDefault();

        mTurretMotor.configFeedbackNotContinuous(true, 10);

        mTurretMotor.configContinuousCurrentLimit(30, 10);
        mTurretMotor.configPeakCurrentLimit(60, 10);
        mTurretMotor.configPeakCurrentDuration(100, 10);
        mTurretMotor.enableCurrentLimit(true);

        mTurretMotor.setSensorPhase(false); // supposed to call setSensorPhase before setInverted?
        mTurretMotor.setInverted(Robot.bot.isTurretInverted);
        setBrakeMode(true);

        if (Robot.bot.turretHasGadgeteerLimits) {
            /*
             * Configured forward and reverse limit switch of Talon to be from a feedback
             * connector and be normally open
             */
            mTurretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                    LimitSwitchNormal.NormallyOpen, 0);
            mTurretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                    LimitSwitchNormal.NormallyOpen, 0);

            // mTurretMotor.configForwardSoftLimitThreshold(minHardLimit, 10);
            // mTurretMotor.configReverseSoftLimitThreshold(maxHardLimit, 10);
            mTurretMotor.configForwardSoftLimitEnable(false, 10);
            mTurretMotor.configReverseSoftLimitEnable(false, 10);
        } else {

            mCwLimit = new DigitalInput(Robot.bot.kTurretCwLimitID); // FIXME: check with RobotHardware
            mCcwLimit = new DigitalInput(Robot.bot.kTurretCCwLimitID);

            mTurretMotor.configForwardSoftLimitThreshold(maxHardLimit, 10);
            mTurretMotor.configReverseSoftLimitThreshold(minHardLimit, 10);
            mTurretMotor.configForwardSoftLimitEnable(true, 10);
            mTurretMotor.configReverseSoftLimitEnable(true, 10);

        }
        // mTurretMotor.configOpenloopRamp(0.20, 10);
        // mTurretMotor.configClosedloopRamp(0.20, 10);

        mTurretMotor.configPeakOutputForward(1.0, 10);
        mTurretMotor.configPeakOutputReverse(-1.0, 10);
        mTurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, 10);
        mTurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 10);
        mTurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 10, 10);
        mTurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, 10);
        mTurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

        int position = mTurretMotor.getSensorCollection().getPulseWidthPosition();// get absolute position

        // seed quadencoder with absolute position
        mTurretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mTurretMotor.getSensorCollection().setQuadraturePosition(position, 100);

        // configurationOne();
        configMagicPIDs(); // check ctre documentation for value tuning

        // setOpenLoop(0.0);

    }

    public static Turret getInstance() {
        return instance == null ? instance = new Turret() : instance;
    }

    public TurretStates getCurrentState() {
        return currentState;
    }

    public void setTurretState(TurretStates s) {
        currentState = s;
    }

    public void setHomePosition(HomePositions position) {
        this.homePosition = position;
    }

    public boolean getCwLimitPressed() {
        if (Robot.bot.turretHasGadgeteerLimits) {
            return mTurretMotor.isFwdLimitSwitchClosed() == 1 ? true : false;
        } else {
            return !mCwLimit.get();
        }
    }

    public boolean getCcwLimitPressed() {
        if (Robot.bot.turretHasGadgeteerLimits) {
            return mTurretMotor.isRevLimitSwitchClosed() == 1 ? true : false;
        } else {
            return !mCcwLimit.get();
        }

    }

    public void resetEncoderZero(boolean isCcw) {
        // FIXME: change position
        turretEncoderZero = periodicIO.position + (isCcw ? Robot.bot.kTurretMaxCCWEncLimit * 4096.0 / 360.0
                : Robot.bot.kTurretMaxCWEncLimit * 4096.0 / 360.0);

        mTurretMotor.configForwardSoftLimitThreshold(turretAngleToEncUnits(maxHardLimit), 10);
        mTurretMotor.configReverseSoftLimitThreshold(turretAngleToEncUnits(minHardLimit), 10);
    }

    private void configurationOne() {
        mTurretMotor.selectProfileSlot(0, 0);
        mTurretMotor.config_kP(0, Robot.bot.kTurretKP, 10);
        mTurretMotor.config_kI(0, Robot.bot.kTurretKI, 10);
        mTurretMotor.config_kD(0, Robot.bot.kTurretKD, 10);
        mTurretMotor.config_kF(0, Robot.bot.kTurretKFF, 10);
        mTurretMotor.config_IntegralZone(0, Robot.bot.kTurretIzone, 10);
        mTurretMotor.configMotionCruiseVelocity(Robot.bot.kTurretCruiseV, 10);
        mTurretMotor.configMotionAcceleration(Robot.bot.kTurretMA, 10);

        mTurretMotor.config_kP(1, Robot.bot.kTurretLockedKP, 10);
        mTurretMotor.config_kI(1, Robot.bot.kTurretLockedKI, 10);
        mTurretMotor.config_kD(1, Robot.bot.kTurretLockedKD, 10);
        mTurretMotor.config_kF(1, Robot.bot.kTurretLockedKFF, 10);

        mTurretMotor.config_kP(2, Robot.bot.kTurretSnapKP, 10);
        mTurretMotor.config_kI(2, Robot.bot.kTurretSnapKI, 10);
        mTurretMotor.config_kD(2, Robot.bot.kTurretSnapKD, 10);
        mTurretMotor.config_kF(2, Robot.bot.kTurretSnapKFF, 10);
    }

    public void configMagicPIDs() {
        mTurretMotor.configNominalOutputForward(0, 10);
        mTurretMotor.configNominalOutputReverse(0, 10);
        mTurretMotor.configPeakOutputForward(1, 10);
        mTurretMotor.configPeakOutputReverse(-1, 10);

        mTurretMotor.config_kP(0, Robot.bot.kTurretKP, 10);
        mTurretMotor.config_kI(0, Robot.bot.kTurretKI, 10);
        mTurretMotor.config_kD(0, Robot.bot.kTurretKD, 10);
        mTurretMotor.config_kF(0, Robot.bot.kTurretKFF, 10);
        mTurretMotor.config_IntegralZone(0, Robot.bot.kTurretIzone, 10);
        mTurretMotor.configAllowableClosedloopError(0, degreesToEncUnits(0.5), 10); // better way to set turret
                                                                                    // tolerance?

        mTurretMotor.configMotionCruiseVelocity(Robot.bot.kTurretCruiseV, 10); // 1235 //2515 //0.3
        mTurretMotor.configMotionAcceleration(Robot.bot.kTurretMA, 10);
        // similar to trapezoidal profiling, 0-8
        mTurretMotor.configMotionSCurveStrength(0);

        // 1ms per loop
        mTurretMotor.configClosedLoopPeriod(0, 1, 10);
        mTurretMotor.selectProfileSlot(0, 0);
    }

    public void setBrakeMode(boolean brake) {
        mTurretMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    // public void setTurretAutoHome() {
    // currentState = TurretStates.AUTO_HOME;
    // }

    public boolean isOpenLoop() {
        return currentState == TurretStates.PERCENT_ROTATION;
    }

    public void setOpenLoop(double output) {
        periodicIO.demand = output;
        setTurretState(TurretStates.PERCENT_ROTATION);
    }

    public void moveToTarget() {
        // mTurretMotor.selectProfileSlot(0, 0);

        mTurretMotor.configPeakOutputForward(1.0, 10);
        mTurretMotor.configPeakOutputReverse(-1.0, 10);

        setTurretState(TurretStates.VISION_MOVING);
        // mTurretMotor.config_kF(0, 0.0, 10);
        // mTurretMotor.config_kF(0, 2.56, 10);
    }

    public void setFieldRelative() {
        setTurretState(TurretStates.FIELD_RELATIVE);
    }

    public void lockOnTarget() {
        mTurretMotor.configPeakOutputForward(1.0, 10);
        mTurretMotor.configPeakOutputReverse(-1.0, 10);
        // mTurretMotor.selectProfileSlot(1, 0);
        setTurretState(TurretStates.VISION_LOCKED);
    }

    // public void autonLockOnTarget() {
    // mTurretMotor.config_kF(0, 0.0, 10);
    // setTurretState(TurretStates.AUTON_LOCKED);
    // }

    public void trackFieldForward() {

        // mTurretMotor.config_kF(0, 2.56, 10);
        setTurretState(TurretStates.FIELD_FORWARD);
        // mTurretMotor.config_kF(0, 2.56, 10);
    }

    public void aimToFront() {
        mTurretMotor.configPeakOutputForward(1.0, 10);
        mTurretMotor.configPeakOutputReverse(-1.0, 10);
        setTurretState(TurretStates.TURRET_HOME);
    }

    public void setToBatter() {
        mTurretMotor.configPeakOutputForward(1.0, 10);
        mTurretMotor.configPeakOutputReverse(-1.0, 10);
        // if (Math.abs(periodicIO.demand - periodicIO.position) > 300) { // IF offet is
        // large select snapping PID
        // mTurretMotor.selectProfileSlot(2, 0);
        // } else { // select vision PID
        // mTurretMotor.selectProfileSlot(0, 0);
        // // mTurretMotor.config_kI(0, 0.0);
        // }
        setTurretState(TurretStates.BATTER);
    }

    public void setToPreset(double angleOffset) {
        mTurretMotor.configPeakOutputForward(1.0, 10);
        mTurretMotor.configPeakOutputReverse(-1.0, 10);
        // mTurretMotor.selectProfileSlot(0, 0);
        // if (Math.abs(periodicIO.demand - periodicIO.position) > 300) { // IF offet is
        // large select snapping PID
        // mTurretMotor.selectProfileSlot(2, 0);
        // } else { // select vision PID
        // mTurretMotor.selectProfileSlot(0, 0);
        // // mTurretMotor.config_kI(0, 0.0);
        // }
        periodicIO.demand = turretAngleToEncUnits(angleOffset);
        setTurretState(TurretStates.SHOOTING_PRESET);
    }

    public double getAngle() {
        return encUnitsToTurretAngle(periodicIO.position);
    }

    public double getFieldCentricAngleDegrees() {
        double centric = -getAngle() + Swerve.getInstance().getAngleDegrees();
        centric = centric > 0 ? centric - 360 : centric;
        return centric;
    }

    private double calculateOnTheMoveOffset(double distanceToTarget, Translation2d robotVelocity) {
        double robotVelocityDirectionRad = Math.atan2(robotVelocity.getY(), robotVelocity.getX());
        double fieldCentricTurretAngleRad = Math.toRadians(getFieldCentricAngleDegrees());

        if (fieldCentricTurretAngleRad < 0)
            fieldCentricTurretAngleRad += 2 * Math.PI;

        SmartDashboard.putNumber("Robot Velocity Angle", Math.toDegrees(robotVelocityDirectionRad));
        SmartDashboard.putNumber("Turret FC Angle", Math.toDegrees(fieldCentricTurretAngleRad));

        // Rotation2d robotVelocityDirection = new Rotation2d(robotVelocity.getX(),
        // robotVelocity.getY());
        // Rotation2d fieldCentricTurretAngle =
        // Rotation2d.fromDegrees(getFieldCentricAngleDegrees());

        // Rotation2d between = robotVelocityDirection.minus(fieldCentricTurretAngle);

        double between = robotVelocityDirectionRad - fieldCentricTurretAngleRad;

        SmartDashboard.putNumber("Turret Between Angle", Math.toDegrees(between));

        double tangentialVelocityAngleRadians = (Math.PI / 2) - between;
        double tangentialVelocityMag = robotVelocity.getNorm() * Math.cos(tangentialVelocityAngleRadians);

        SmartDashboard.putNumber("Vt angle", Math.toDegrees(tangentialVelocityAngleRadians));
        SmartDashboard.putNumber("Vt mag", tangentialVelocityMag);

        double angularVelocityRadians = tangentialVelocityMag / Units.inchesToMeters(distanceToTarget);

        SmartDashboard.putNumber("Turret Angular Velocity Radians", angularVelocityRadians);

        double ff = angularVelocityRadians / (Math.PI / 4); // 45 degree max
        ff *= turretTable.getEntry("Turret Offset").getDouble(30.0); // FIXME: Tune this
        return ff;
        // SmartDashboard.putNumber("Caclulated Turret FF", ff);
        // return Math.min(Math.max(-1, ff), 1);
    }

    public double encUnitsToDegrees(double encUnits) {
        return encUnits / 4096.0 * 360.0;
    }

    public int degreesToEncUnits(double degrees) {
        return (int) (degrees / 360.0 * 4096.0);
    }

    public double encUnitsToTurretAngle(int encUnits) {
        return encUnitsToDegrees(encUnits - (int) turretEncoderZero);
    }

    public int turretAngleToEncUnits(double mTurretMotorAngle) {
        return (int) turretEncoderZero + degreesToEncUnits(mTurretMotorAngle);
    }

    public boolean isSensorConnected() {
        int pulseWidthPeriod = mTurretMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
        boolean connected = pulseWidthPeriod != 0;
        if (!connected)
            hasEmergency = true;
        return connected;
    }

    public void onLoop(double timestamp) {
        if (mTurretMotor.getStatorCurrent() > 80) {
            DriverStation.reportError("Turret current high", false);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.position = (int) mTurretMotor.getSelectedSensorPosition(0);
        periodicIO.velocity = (int) mTurretMotor.getSelectedSensorVelocity(0);
        periodicIO.voltage = mTurretMotor.getMotorOutputVoltage();
        periodicIO.current = mTurretMotor.getStatorCurrent();
    }

    private boolean isStalled() {
        double current = mTurretMotor.getStatorCurrent();
        double vel = mTurretMotor.getSelectedSensorVelocity(0);
        return stallDebouncer.calculate((current > 25.0) && (vel <= 1));
    }

    public boolean encoderTargetInRange() {
        return (periodicIO.position <= periodicIO.demand + degreesToEncUnits(0.5)
                || periodicIO.position >= periodicIO.demand - degreesToEncUnits(0.5));
        // return Math.abs(mTurretMotor.getActiveTrajectoryPosition() -
        // periodicIO.position) <= 10;

    }

    public boolean isTurretOnTarget() {
        return (encoderTargetInRange() && LimelightProcessor.getInstance().isTargetVisible());
        // return LimelightProcessor.getInstance().isTargetVisible()
        // && (LimelightProcessor.getInstance().getPowerPortTurretError() < 1.0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        double desiredAngle;
        double desiredAngleLL;
        double ff = 0.0;

        double x = Swerve.getInstance().getPoseMeters().getX();
        double y = Swerve.getInstance().getPoseMeters().getY();
        double currentFieldCentricAngle = Math.toRadians(getFieldCentricAngleDegrees());
        double metersDistance = Units.inchesToMeters(LimelightProcessor.getInstance().getDistFromHub());
        targetX = curTargetX - x;
        targetY = curTargetY - y;
        targetDeg = Math.toDegrees(Math.atan2(targetY, targetX));
        targetDeg = targetDeg > 0 ? targetDeg - 360 : targetDeg;
        if ((getCwLimitPressed() || getCcwLimitPressed()) && !edu.wpi.first.wpilibj.RobotState.isTest()) {
            reseedEncoderValues();
            // if (periodicIO.demand < minHardLimit &&
            // !LimelightProcessor.getInstance().isTargetVisible())
            // periodicIO.demand = periodicIO.position + 20;
            // if (periodicIO.demand > maxHardLimit &&
            // !LimelightProcessor.getInstance().isTargetVisible())
            // periodicIO.demand = periodicIO.position - 20;
            // aimToFront();
        }

        // if (isStalled()) {
        // mTurretMotor.set(ControlMode.PercentOutput, 0);
        // }

        SmartDashboard.putString("Turret State", currentState.toString());
        switch (currentState) {
            case PERCENT_ROTATION:

                if (periodicIO.position < maxHardLimit - degreesToEncUnits(25) && periodicIO.demand > 0) {
                    mTurretMotor.set(ControlMode.PercentOutput, periodicIO.demand);
                } else if (periodicIO.position > minHardLimit + degreesToEncUnits(25) && periodicIO.demand < 0) {
                    mTurretMotor.set(ControlMode.PercentOutput, periodicIO.demand);
                } else {
                    mTurretMotor.set(ControlMode.PercentOutput, 0.0);
                }

                break;

            case VISION_LOCKED:
                if (Math.abs(LimelightProcessor.getInstance().getTX()) >= 2) { // TODO: need to add
                    // // error
                    // threshold to transition
                    // b/t locking and moving

                    moveToTarget(); // PIDs
                    break;
                }

                curTargetX = x + metersDistance * Math.cos(currentFieldCentricAngle);
                curTargetY = y + metersDistance * Math.sin(currentFieldCentricAngle);

                if (!LimelightProcessor.getInstance().isTargetVisible()) {
                    setFieldRelative();
                    break;
                }

                desiredAngle = LimelightProcessor.getInstance().getHubError() + getAngle();
                desiredAngleLL = desiredAngle;
                periodicIO.demand = turretAngleToEncUnits(desiredAngle);

                mTurretMotor.set(ControlMode.Position, periodicIO.demand,
                        DemandType.ArbitraryFeedForward, 0.0);
                break;
            case VISION_MOVING:
                if (!LimelightProcessor.getInstance().isTargetVisible()) {
                    setFieldRelative();
                    break;
                }
                if (Turret.getInstance().isTurretOnTarget() && Math.abs(LimelightProcessor.getInstance().getTX()) < 1) {
                    mTurretMotor.config_kI(0, 0.0);
                    curTargetX = x + metersDistance * Math.cos(currentFieldCentricAngle);
                    curTargetY = y + metersDistance * Math.sin(currentFieldCentricAngle);
                } else {
                    mTurretMotor.config_kI(0, Robot.bot.kTurretKI);
                }

                // if (Math.abs(LimelightProcessor.getInstance().getHubError()) <= 1.0) {
                // setTurretState(TurretStates.VISION_LOCKED);
                // break;
                // }

                double d = LimelightProcessor.getInstance().getDistFromHub();
                double rawTX = LimelightProcessor.getInstance().getHubError();
                double tx = Math.toRadians(rawTX);
                fixedTx = Math.toDegrees(
                        Math.atan((d * Math.sin(tx)) / (d * Math.cos(tx) - limelightCenterOffset)));

                desiredAngle = (LimelightProcessor.getInstance().getHubError() * (1.0 - Math.abs((rawTX / 150))))
                        + getAngle();
                desiredAngleLL = desiredAngle;

                Translation2d swerveVel = Swerve.getInstance().getVelocity().getTranslation();
                // double offset =
                // calculateOnTheMoveOffset(LimelightProcessor.getInstance().distFromTarget(),
                // swerveVel);
                // turretTable.getEntry("Turret Offset Val")
                // .setNumber(offset);
                // @formatter:off
                // desiredAngle -= swerveVel.getNorm() > 0.1 ? offset : 0;
                turretTable.getEntry("Offset amount").setNumber(desiredAngleLL-desiredAngle);
                
                periodicIO.demand = turretAngleToEncUnits(desiredAngle);
                
                double arbff = 0;
                // ROTATION COMPENSATION FF
                if (Swerve.getInstance().getGyroRotationalVelocity().getDegrees() < 5
                        && Math.abs(LimelightProcessor.getInstance().getHubError()) < 15) {
                    // double proportionalFF = .4 * (LimelightProcessor.getInstance().getHubError() / 20);
                    // double ffSign = Math.signum(proportionalFF);
                    // arbff = Math.min(.4, Math.abs(proportionalFF));
                    // arbff *= ffSign;
                } else {
                    double proportionalFF = .2 *
                            (Swerve.getInstance().getGyroRotationalVelocity().getDegrees() / 200);
                    double ffSign = Math.signum(proportionalFF);
                    arbff = Math.min(.2, Math.abs(proportionalFF));
                    arbff *= ffSign;

                }
                if(periodicIO.demand < minHardLimit || periodicIO.demand > maxHardLimit){
                    periodicIO.demand = periodicIO.position;
                }
                // periodicIO.demand = turretEncoderZero
                // + degreesToEncUnits(turretTable.getEntry("daSanjith Offset").getDouble(0));
                // mTurretMotor.set(ControlMode.Position, periodicIO.demand,
                //         DemandType.ArbitraryFeedForward, arbff);
                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
                break;

            case AUTO_HOME: //unused
                mTurretMotor.configForwardSoftLimitEnable(false, 10);
                mTurretMotor.configReverseSoftLimitEnable(false, 10);
                periodicIO.demand = -.4;
                if (getCwLimitPressed() || getCcwLimitPressed()) {
                    mTurretMotor.configForwardSoftLimitEnable(true, 10);
                    mTurretMotor.configReverseSoftLimitEnable(true, 10);
                    trackFieldForward();
                    break;
                }
                mTurretMotor.set(ControlMode.PercentOutput, periodicIO.demand);
                break;
            case TURRET_HOME:
                if (LimelightProcessor.getInstance().isTargetVisible()) {
                    moveToTarget();
                    break;
                }
                periodicIO.demand = turretEncoderHome;
                // periodicIO.demand = (this.homePosition == HomePositions.RIGHT ? turretEncoderHome : (this.homePosition == HomePositions.BACK ? turretEncoderBack : turretEncoderBatter));
                mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);       
                break;
            case TURRET_BACK:
                // mTurretMotor.configPeakOutputForward(1.0, 10);
                // mTurretMotor.configPeakOutputReverse(-1.0, 10);
                if (LimelightProcessor.getInstance().isTargetVisible()) {
                    moveToTarget();
                    break;
                }
                periodicIO.demand = turretEncoderBack;

                // periodicIO.demand = (this.homePosition == HomePositions.RIGHT ? turretEncoderHome : (this.homePosition == HomePositions.BACK ? turretEncoderBack : turretEncoderBatter));
                mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);       
                break;
            case BATTER:
                
                periodicIO.demand = turretEncoderBatter - 10;

                // mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);
                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
                if(StateEngine.getInstance().getRobotState() == RobotStates.CLIMBING){
                    //stay at batter
                }else{
                    if(!StateEngine.getInstance().isPresetOverride()) {
                        moveToTarget();
                    }
                }
                break;
            case SHOOTING_PRESET:
               double target = periodicIO.demand;
                if (target < minHardLimit){
                    target += 4096;
                }
                else if(target > maxHardLimit){
                    target -=4096;
                } 

                periodicIO.demand = target;
                if(periodicIO.demand < minHardLimit || periodicIO.demand > maxHardLimit){
                    periodicIO.demand = periodicIO.position;
                }
                
                // mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);
                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
                break;
            case POSITION:
                
                // mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);
                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
                
                break;
            case FIELD_RELATIVE:

                if (LimelightProcessor.getInstance().isTargetVisible()){
                    moveToTarget();
                    break;
                }

                // Don't shift between profiles if using motion magic
                // if(Math.abs(periodicIO.demand - periodicIO.position) > 300){ // IF offet is large select snapping PID
                //     mTurretMotor.selectProfileSlot(2, 0);
                //     mTurretMotor.configPeakOutputForward(0.8, 10);
                //     mTurretMotor.configPeakOutputReverse(-0.8, 10);
                // }else{ // select vision PID
                //     mTurretMotor.selectProfileSlot(0, 0);
                //     mTurretMotor.configPeakOutputForward(1.0, 10);
                //     mTurretMotor.configPeakOutputReverse(-1.0, 10);
                // }
                
                SmartDashboard.putNumber("Target X", targetX);
                SmartDashboard.putNumber("Target Y", targetY);
             
                double targetOffset = getFieldCentricAngleDegrees() - targetDeg;
                targetDemand = periodicIO.position + degreesToEncUnits(targetOffset);

                if (targetDemand < minHardLimit) 
                    targetDemand += 4096;
                else if(targetDemand > maxHardLimit) 
                    targetDemand -=4096;
                // System.out.println("TARGET DEMAND: " + targetDemand);


                periodicIO.demand = targetDemand;
                if(periodicIO.demand < minHardLimit || periodicIO.demand > maxHardLimit){
                    periodicIO.demand = periodicIO.position;
                }
                // mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);

                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);                
                break;
            case DISABLED:
                mTurretMotor.set(ControlMode.PercentOutput, 0);
                break;

        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {

    }

    public void initTelemetry() {
        // turretTableList.withPosition(10, 0).withSize(2, 5);

        turretTableList.addBoolean("Turret Telemetry Toggle",
                () -> outputtingTelemetry()).withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);
        turretTableList.addBoolean("Turret Telemetry",
                () -> outputtingTelemetry()).withSize(2, 1);
        turretTableList.addString("Turret State",
                () -> currentState.toString()).withSize(1, 1);

        turretTableList.addNumber("Turret P", () -> turretTable.getEntry("Turret P").getDouble(Robot.bot.kTurretKP))
                .withSize(1, 1);
        turretTableList.addNumber("Turret I", () -> turretTable.getEntry("Turret I").getDouble(Robot.bot.kTurretKI))
                .withSize(1, 1);
        turretTableList.addNumber("Turret D", () -> turretTable.getEntry("Turret D").getDouble(Robot.bot.kTurretKD))
                .withSize(1, 1);
        turretTableList.addNumber("Turret Cruise Velocity", () -> turretTable.getEntry("Turret Cruise Velocity").getDouble(Robot.bot.kTurretCruiseV))
                .withSize(1, 1);
        turretTableList.addNumber("Turret Max Accel", () -> turretTable.getEntry("Turret Max Accel").getDouble(Robot.bot.kTurretMA))
                .withSize(1, 1);
    

        // turretTableList.addNumber("Turret Lock P", () -> turretTable.getEntry("Turret Lock P").getDouble(Robot.bot.kTurretLockedKP))
        //         .withSize(1, 1);
        // turretTableList.addNumber("Turret Lock I", () -> turretTable.getEntry("Turret Lock I").getDouble(Robot.bot.kTurretLockedKI))
        //         .withSize(1, 1);
        // turretTableList.addNumber("Turret Lock D", () -> turretTable.getEntry("Turret Lock D").getDouble(Robot.bot.kTurretLockedKD))
        //         .withSize(1, 1);

        // turretTableList.addNumber("Turret Snap P", () -> turretTable.getEntry("Turret Snap P").getDouble(Robot.bot.kTurretSnapKP))
        //         .withSize(1, 1);
        // turretTableList.addNumber("Turret Snap I", () -> turretTable.getEntry("Turret Snap I").getDouble(Robot.bot.kTurretSnapKI))
        //         .withSize(1, 1);
        // turretTableList.addNumber("Turret Snap D", () -> turretTable.getEntry("Turret Snap D").getDouble(Robot.bot.kTurretSnapKD))
        //         .withSize(1, 1);

        turretTableList.addNumber("Turret Correct Tx", () -> fixedTx).withSize(1,1);

        turretTableList.addNumber("Turret Locking Threshold", () -> turretTable.getEntry("Turret Locking Threshold").getDouble(kTurretLockingThreshold))
            .withSize(1, 1);

        turretTableList
                .addNumber("Turret FF", () -> turretTable.getEntry("Turret FF").getDouble(Robot.bot.kTurretKFF))
                .withSize(1, 1);

        turretTableList
                .addNumber("Turret Offset Deg", () -> turretTable.getEntry("Turret Offset").getDouble(10))
                .withSize(1, 1);

        turretTableList
            .addNumber("Turret ANgle Output", this::getAngle)
            .withSize(1, 1);
        
        turretTableList
            .addNumber("Turret Field Centric Angle", this::getFieldCentricAngleDegrees)
            .withSize(1, 1);

        turretTableList.addNumber("Angular Velocity to Target",
                () -> Math.toDegrees(calculateOnTheMoveOffset(LimelightProcessor.getInstance().getDistFromHub(),
                        Swerve.getInstance().getVelocity().getTranslation())))
                .withSize(1, 1);

        turretTableList.addNumber("Turret Trajectory Target", () -> mTurretMotor.getActiveTrajectoryPosition());
        turretTableList.addNumber("Turret Trajectory Velocity", () -> mTurretMotor.getActiveTrajectoryVelocity());
        turretTableList.addNumber("Turret Closed Loop Error", () -> mTurretMotor.getClosedLoopError());
        turretTableList.addNumber("Turret Sensor Position", () -> mTurretMotor.getSelectedSensorPosition());
        turretTableList.addNumber("Turret Sensor Velocity", () -> mTurretMotor.getSelectedSensorVelocity());
        turretTableList.addBoolean("Relative w limit", () -> isFieldRelativeWithinLimits());
        turretTableList.addBoolean("Turret Field Relative Within Limits", () -> isFieldRelativeWithinLimits());
        turretTableList.addNumber("TURRET TARGET", () -> targetDemand);

    }

    @Override
    public void outputTelemetry() {
        if (outputtingTelemetry()) {
            turretTable.getEntry("Turret Temperature").setNumber(mTurretMotor.getTemperature());
            turretTable.getEntry("Turret Desired ENC units").setNumber(periodicIO.demand);
            turretTable.getEntry("Turret current ENC units").setNumber(periodicIO.position);
            turretTable.getEntry("IZONE APPLLIED INPUT").setNumber(mTurretMotor.getIntegralAccumulator());
            turretTable.getEntry("Turret CLICKS Error")
                    .setNumber(periodicIO.demand - periodicIO.position);
            turretTable.getEntry("Turret Angle").setNumber(getAngle());
            turretTable.getEntry("Turret Cw Limit").setBoolean(getCwLimitPressed());
            turretTable.getEntry("Turret cCw Limit").setBoolean(getCcwLimitPressed());
            turretTable.getEntry("Turret Power").setNumber(mTurretMotor.getMotorOutputPercent());
            turretTable.getEntry("Turret Home State").setString(homePosition.toString());
            turretTable.getEntry("Turret Current").setDouble(mTurretMotor.getStatorCurrent());
            turretTable.getEntry("Is Stalled").setBoolean(isStalled());



        }

        // SmartDashboard.putNumber("Turret Selected Sensor Position", mTurretMotor.getSelectedSensorVelocity());

        // SmartDashboard.putNumber("Turret Angle", getAngle());
        // // SmartDashboard.putNumber("Turret Field Centric", getFieldCentricAngle());
        // // SmartDashboard.putNumber("Turret Angle To Target",
        // getFieldCentricAngle());
        // // SmartDashboard.putNumber("Turret Calculated Skew",
        // getCorrectiveSkewAngle());
        // SmartDashboard.putNumber("Turret Encoder Zero", turretEncoderZero);
        // SmartDashboard.putString("Turret State", getCurrentState().name());
        // SmartDashboard.putBoolean("Turret Cw Limit", getCwLimitPressed());
        // SmartDashboard.putBoolean("Turret Ccw Limit", getCcwLimitPressed());

    }

    int kTimeoutMs = 100;

    public void updatePIDValues() {
        mTurretMotor.config_kP(0, turretTable.getEntry("Turret P").getDouble(0.0), 10);
        mTurretMotor.config_kI(0, turretTable.getEntry("Turret I").getDouble(0.0), 10);
        mTurretMotor.config_kD(0, turretTable.getEntry("Turret D").getDouble(0.0), 10);
        mTurretMotor.configMotionCruiseVelocity(turretTable.getEntry("Turret Cruise Velocity").getDouble(0.0), 10);
        mTurretMotor.configMotionAcceleration(turretTable.getEntry("Turret Max Accel").getDouble(0.0), 10);

        // mTurretMotor.config_kP(1, turretTable.getEntry("Turret Lock P").getDouble(0.0), kTimeoutMs);
        // mTurretMotor.config_kI(1, turretTable.getEntry("Turret Lock I").getDouble(0.0), kTimeoutMs);
        // mTurretMotor.config_kD(1, turretTable.getEntry("Turret Lock D").getDouble(0.0), kTimeoutMs);

        // mTurretMotor.config_kP(2, turretTable.getEntry("Turret Snap P").getDouble(0.0), kTimeoutMs);
        // mTurretMotor.config_kI(2, turretTable.getEntry("Turret Snap I").getDouble(0.0), kTimeoutMs);
        // mTurretMotor.config_kD(2, turretTable.getEntry("Turret Snap D").getDouble(0.0), kTimeoutMs);

        kTurretLockingThreshold = turretTable.getEntry("Turret Locking Threshold").getDouble(75.0);
        // mTurretMotor.config_kF(0, turretTable.getEntry("Turret FF").getDouble(0.0), kTimeoutMs);
    }

    private boolean outputtingTelemetry() {
        return turretTable.getEntry("Turret Telemetry Toggle").getBoolean(false);
    }

    public boolean isFieldRelativeWithinLimits() {
        return targetDemand > minHardLimit && targetDemand < maxHardLimit;
    }

    public double getTargetFieldRelativeAngle() {
        return targetDeg;
    }

    public void setManual(double angleOffset) { 
        // mTurretMotor.configPeakOutputForward(1.0, 10);
        // mTurretMotor.configPeakOutputReverse(-1.0, 10);
        // if (Math.abs(periodicIO.demand - periodicIO.position) > 300) { // IF offet is large select snapping PID
        //     mTurretMotor.selectProfileSlot(2, 0);
        // } else { // select vision PID
        //     mTurretMotor.selectProfileSlot(0, 0);
        //     // mTurretMotor.config_kI(0, 0.0);
        // }
        periodicIO.demand = turretAngleToEncUnits(angleOffset);
        setTurretState(TurretStates.POSITION);
    }

    private void reseedEncoderValues(){
        if(getCwLimitPressed()){
            maxHardLimit = mTurretMotor.getSelectedSensorPosition();
            minHardLimit = maxHardLimit-kTurretEncRange;
            turretEncoderZero = minHardLimit + kEncZeroOffset;
            turretEncoderBatter = minHardLimit + kEncBatterOffset;
            turretEncoderHome = minHardLimit + kEncHomeOffset;
        }
        else if(getCcwLimitPressed()){
            minHardLimit = mTurretMotor.getSelectedSensorPosition();
            maxHardLimit = minHardLimit+kTurretEncRange;
            turretEncoderZero = minHardLimit + kEncZeroOffset;
            turretEncoderBatter = minHardLimit + kEncBatterOffset;
            turretEncoderHome = minHardLimit + kEncHomeOffset;
        }
     
    }


    public boolean checkSystem() {
        double currentMinimum = 0.5;
        double currentMaximum = 20.0;

        boolean passed = true;

        if (!isSensorConnected()) {
            System.out.println("Turret sensor is not connected, connect and retest");
            return false;
        }

        double startingEncPosition = mTurretMotor.getSelectedSensorPosition(0);
        mTurretMotor.set(ControlMode.PercentOutput, 3.0 / 12.0);
        Timer.delay(1.0);
        double current = mTurretMotor.getStatorCurrent();
        mTurretMotor.set(ControlMode.PercentOutput, 0.0);
        if (Math.signum(mTurretMotor.getSelectedSensorPosition(0) - startingEncPosition) != 1.0) {
            System.out.println("Turret needs to be reversed");
            passed = false;
        }
        if (current < currentMinimum) {
            System.out.println("Turret current too low: " + current);
            passed = false;
        } else if (current > currentMaximum) {
            System.out.println("Turret current too high: " + current);
            passed = false;
        }

        return passed;
    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {

    }

    public static class PeriodicIO {
        // Inputs
        public int position;
        public int velocity;
        public double voltage;
        public double current;

        // Outputs
        public double demand;
    }
}
