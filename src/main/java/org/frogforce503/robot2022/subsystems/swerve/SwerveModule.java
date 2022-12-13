package org.frogforce503.robot2022.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;

import org.frogforce503.lib.drivers.TalonFXWrapper;
import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.subsystems.Subsystem;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.ModuleLocation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {

    private static final double kTurnEncoderClicksperRevolution = 2048; // 3178
    private static final double turnGearRatio = 7.0 / 150.0; // Unitless (FF 6 is 15) (mk4 is 12.8) (14.28 for mk4i)
    // (22.162) (21.4)
    private static final double kTurnMotorEncoderPositionCoefficientRadiansPerCount = (2.0 * Math.PI
            / kTurnEncoderClicksperRevolution)
            * turnGearRatio;
    private static final double kTurnMotorEncoderVelocityCoefficient = kTurnMotorEncoderPositionCoefficientRadiansPerCount
            * 10;

    public static final double MAX_DRIVE_SPEED_METERS_SEC = Units.feetToMeters(16.3); // 11 ft per second approx = 3.3
                                                                                      // m/s
                                                                                      // //FIXME

    private static final int kSlotIdx = 0;
    private static final int kTimeoutMs = 100;

    private final double driveGearRatio = 6.75; // Unitless (FOR L1 on SDS Mk4i. FF 6 is 7.125) //FIXME
    private static final double kWheelDiameter = Robot.bot.wheelDiameter;
    private final double CIRCUMFERENCE = (Math.PI * kWheelDiameter); // meters for 1 rotation
    private final double CLICKS_PER_REV_DRIVE_MOTOR = 2048;
    private final double metersPerSecondToDriveClicksPer100Millisecond = ((CLICKS_PER_REV_DRIVE_MOTOR / CIRCUMFERENCE)
            * driveGearRatio) / 10; // ((1 rotation / circumference (neters)) * 2048) / 10
    // (because it is 100ms)

    double resetIteration = 0;
    double preResetCanCoder = 0;
    private static final int ENCODER_RESET_ITERATIONS = (int) (50);

    // reset allowed if moving less than .5 degrees persecond
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    private int resetTimes = 0;

    public TalonFXWrapper driveMotor;
    public TalonFXWrapper turnMotor;
    private CANCoder turnEncoder;

    Rotation2d currentlyTargetingAngle = Rotation2d.fromDegrees(503.503503503);

    private double _simTurnSetpoint;
    private double _simDriveSetpoint;
    private double _simTurnEncoderPosition;
    private double _simDriveEncoderVelocity;
    private final double _simTurnAngularVelocity = 20;
    private final double _simDriveAcceleration = 5.0;

    public String moduleName;
    private String debugName;
    public String locationName;
    int driveMotorID;
    int turnMotorID;
    int turnEncoderID;
    double turn_kP;
    double turn_kI;
    double turn_kD;
    double turn_kF;
    double drive_kP;
    double drive_kI;
    double drive_kD;
    double drive_kF;
    double absoluteZeroDegrees;
    boolean driveInvert;
    boolean turnInvert;

    // private static final double drive_kP = 0.1;
    // private static final double drive_kI = 0.001;
    // private static final double drive_kD = 5;
    // private static final double drive_kF = 1023.0 / 20660.0;

    // private static final double turn_kP = 0.2;
    // private static final double turn_kI = 0.0;
    // private static final double turn_kD = 4.8;
    // private static final double turn_kF = 0.0;

    public SwerveModule(String moduleName, ModuleLocation location) {
        this.moduleName = moduleName;
        this.debugName = location.name() + "(" + moduleName + ")";
        this.locationName = location.name();
        SwerveModuleLoader.getInstance().setupModule(this);
        // try {
        // loadFromJSON(moduleName + ".json");
        // } catch (Exception e) {
        // System.out.println("FAILED TO CREATE MODULE " + moduleName);
        // }

        driveMotor = new TalonFXWrapper(driveMotorID);
        turnMotor = new TalonFXWrapper(turnMotorID);
        turnEncoder = new CANCoder(turnEncoderID);

        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();
        turnEncoder.configFactoryDefault();

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Coast);

        // configure drive motor
        driveMotor.setInverted(driveInvert);
        // driveMotor.setSensorPhase(driveEncInvert);

        // setup velocity pid
        driveMotor.configNeutralDeadband(0.001);
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        // Config the peak and nominal outputs
        driveMotor.configNominalOutputForward(0, kTimeoutMs);
        driveMotor.configNominalOutputReverse(0, kTimeoutMs);
        driveMotor.configPeakOutputForward(1, kTimeoutMs);
        driveMotor.configPeakOutputReverse(-1, kTimeoutMs);

        // Config the Velocity closed loop gains in slot0
        driveMotor.config_kP(kSlotIdx, drive_kP, kTimeoutMs);
        driveMotor.config_kI(kSlotIdx, drive_kI, kTimeoutMs);
        driveMotor.config_kD(kSlotIdx, drive_kD, kTimeoutMs);
        driveMotor.config_kF(kSlotIdx, drive_kF, kTimeoutMs);

        driveMotor.configClosedloopRamp(0.1);

        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 55, 1.0));

        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);

        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        turnMotor.selectProfileSlot(kSlotIdx, 0);
        turnMotor.config_kF(kSlotIdx, turn_kF, kTimeoutMs);
        turnMotor.config_kP(kSlotIdx, turn_kP, kTimeoutMs);
        turnMotor.config_kI(kSlotIdx, turn_kI, kTimeoutMs);
        turnMotor.config_kD(kSlotIdx, turn_kD, kTimeoutMs);
        turnMotor.setInverted(true);
        turnMotor.setSensorPhase(false);
        turnMotor.configAllowableClosedloopError(0,
                Math.toRadians(0.5) / kTurnMotorEncoderPositionCoefficientRadiansPerCount,
                kTimeoutMs); // previously 50 for mk6 then 36 for mk4i

        turnMotor.configMotionCruiseVelocity(15000, kTimeoutMs);
        turnMotor.configMotionAcceleration(6000, kTimeoutMs);

        turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 55, 1.0));

        turnEncoder.configMagnetOffset(0, 0);

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        System.out.println(moduleName);
        System.out.println(turnEncoder.getAbsolutePosition());

        turnEncoder.configMagnetOffset(absoluteZeroDegrees, 0);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        System.out.println(turnEncoder.getAbsolutePosition());
        turnEncoder.configSensorDirection(false);

        turnMotor.setSelectedSensorPosition(
                (Math.toRadians(turnEncoder.getAbsolutePosition()))
                        / kTurnMotorEncoderPositionCoefficientRadiansPerCount);

        // turnEncoder.setPosition(turnEncoder.getAbsolutePosition());

        if (RobotBase.isSimulation())
            this.simulationInit();
    }

    public void configure(int driveMotorID, int turnMotorID, int turnEncoderID, double turn_kP, double turn_kI,
            double turn_kD,
            double turn_kF, double drive_kP, double drive_kI, double drive_kD, double drive_kF,
            double absoluteZeroDegrees,
            boolean driveInvert,
            boolean turnInvert) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.turnEncoderID = turnEncoderID;
        this.turn_kP = turn_kP;
        this.turn_kI = turn_kI;
        this.turn_kD = turn_kD;
        this.turn_kF = turn_kF;
        this.drive_kP = drive_kP;
        this.drive_kI = drive_kI;
        this.drive_kD = drive_kD;
        this.drive_kF = drive_kF;
        this.absoluteZeroDegrees = absoluteZeroDegrees;
        this.driveInvert = driveInvert;
        this.turnInvert = turnInvert;
    }

    public void enableBrakeMode(boolean enable) {
        driveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    // return module rotation in degrees
    public double getRotationAngle() {
        // return (((RobotBase.isReal() ? turnEncoder.getPosition() :
        // _simTurnEncoderPosition) - absoluteZeroDegrees)
        // % 360);
        double outputAngleRad = (turnMotor.getSelectedSensorPosition()
                * kTurnMotorEncoderPositionCoefficientRadiansPerCount);

        if (RobotBase.isSimulation())
            outputAngleRad = _simTurnEncoderPosition;

        outputAngleRad %= Math.PI * 2.0;

        if (outputAngleRad < 0.0)
            outputAngleRad += Math.PI * 2.0;

        return Math.toDegrees(outputAngleRad);
    }

    public Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees(getRotationAngle());
    }

    public void setDriveMotor(ControlMode controlMode, double outputValue) {
        if (controlMode == ControlMode.Velocity)
            outputValue *= metersPerSecondToDriveClicksPer100Millisecond;

        driveMotor.set(controlMode, outputValue);
        if (RobotBase.isSimulation())
            _simDriveSetpoint = outputValue / metersPerSecondToDriveClicksPer100Millisecond;
    }

    public void outputTelemetry() {
        double curAngle = getRotationAngle();
        SmartDashboard.putNumber(this.locationName + " Calculated Angle", curAngle);
        // SmartDashboard.putNumber(this.locationName + " Drive Velocity",
        // getVelocity());
        // SmartDashboard.putNumber(this.locationName + " CanCoder Angle",
        // turnEncoder.getPosition());

        if (Constants.kOutputTelemetry) {
            SmartDashboard.putNumber(this.locationName + " Absolute CanCoder Angle",
                    turnEncoder.getAbsolutePosition());
            SmartDashboard.putNumber(this.locationName + " FALCON ANGLE CALCULATION ERROR DEGREES",
                    curAngle - turnEncoder.getAbsolutePosition());
            SmartDashboard.putNumber(this.locationName + " Turn Motor Current Draw",
                    turnMotor.getStatorCurrent());
            SmartDashboard.putNumber(this.locationName + " Drive Motor Current Draw",
                    driveMotor.getStatorCurrent());
            SmartDashboard.putNumber(this.locationName + "Drive Motor Speed", getVelocity());
            SmartDashboard.putNumber(this.locationName + "Drive Motor Target", driveMotor.getClosedLoopTarget());
        }
    }

    // public void resetSensors(int num) {
    // turnMotor.setSelectedSensorPosition(num);
    // }

    private void simulationInit() {
        // _simTurnController = new FrogPIDF(2.0, 0.0, 0.032, 0.0,
        // org.frogforce503.lib.util.FrogPIDF.ControlMode.Position_Control);
        // _simTurnController.setTolerance(1);
        // _simDriveController = new FrogPIDF(2, 0.0, 0.0, 0.0,
        // org.frogforce503.lib.util.FrogPIDF.ControlMode.Velocity_Control);
        absoluteZeroDegrees = 0;
    }

    public double getRezeroValue() {
        turnEncoder.configMagnetOffset(0, 0);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double rezeroValue = turnEncoder.getAbsolutePosition();

        turnEncoder.configMagnetOffset(absoluteZeroDegrees, 0);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return rezeroValue;
    }

    public void simulationPeriodic() {
        double turnError = _simTurnSetpoint - getRotationAngle();
        double velocityError = _simDriveSetpoint - getVelocity();
        // System.out.println(_simTurnSetpoint);
        // // double turnStep = Math.signum(turnError) * Math.min(Math.abs(turnError),
        // // _simTurnAngularVelocity);

        // // double turnOutput = _simTurnController.calculateOutput(getRotationAngle(),
        // // false)
        // // * (_simTurnAngularVelocity + ((Math.random() <= 0.5 ? -1 : 1) *
        // Math.random()
        // // * 3));
        // // double driveOutput = _simDriveController.calculateOutput(getVelocity(),
        // // false) * _simDriveVelocity;

        // // SmartDashboard.putNumber("SIM " + moduleName + " TURN OUTPUT", turnStep);
        // // SmartDashboard.putNumber("SIM " + moduleName + " DRIVE OUTPUT",
        // driveOutput);

        if (Math.abs(turnError) > _simTurnAngularVelocity) {
            _simTurnEncoderPosition += Math.signum(turnError) * _simTurnAngularVelocity;
        } else {
            _simTurnEncoderPosition = _simTurnSetpoint;
        }

        if (Math.abs(velocityError) > _simDriveAcceleration) {
            _simDriveEncoderVelocity += Math.signum(velocityError) * _simDriveAcceleration;
        } else {
            _simDriveEncoderVelocity = _simDriveSetpoint;
        }
        // _simDriveEncoderVelocity += driveOutput;
        // _simTurnEncoderPosition += 1;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getRotationAngle()));
    }

    /**
     * @return drive wheel velocity in meters per second
     */
    public double getVelocity() {
        if (RobotBase.isSimulation())
            return _simDriveEncoderVelocity;

        return driveMotor.getSelectedSensorVelocity() / metersPerSecondToDriveClicksPer100Millisecond;
    }

    public void setRotationPosition(Rotation2d referenceAngle) {
        double currentAngleRadians = turnMotor.getSelectedSensorPosition()
                * kTurnMotorEncoderPositionCoefficientRadiansPerCount;
        double referenceAngleRadians = referenceAngle.getRadians();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.

        if (Math.abs(turnMotor.getSelectedSensorVelocity())
                * kTurnMotorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngleRadians = Math.toRadians(turnEncoder.getAbsolutePosition());
                preResetCanCoder = turnEncoder.getAbsolutePosition();
                turnMotor.setSelectedSensorPosition(
                        absoluteAngleRadians / kTurnMotorEncoderPositionCoefficientRadiansPerCount);
                currentAngleRadians = absoluteAngleRadians;
                resetTimes++;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go
        // above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        turnMotor.set(ControlMode.Position,
                adjustedReferenceAngleRadians / kTurnMotorEncoderPositionCoefficientRadiansPerCount);
    }

    @Override
    public void readPeriodicInputs() {
        // if (Math.abs(turnMotor.getSelectedSensorVelocity())
        // * kTurnMotorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY)
        // {
        // if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        // resetIteration = 0;
        // double absoluteAngle = Math.toRadians(turnEncoder.getAbsolutePosition());
        // turnMotor.setSelectedSensorPosition(
        // absoluteAngle / kTurnMotorEncoderPositionCoefficient);
        // resetTimes++;
        // }
        // } else {
        // resetIteration = 0;
        // }
    }

    @Override
    public void stop() {
        if (RobotState.isTest())
            return;
        setSwerveModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        currentlyTargetingAngle = Rotation2d.fromDegrees(503.503503503);
    }

    public void setSwerveModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getModuleRotation());
        // if (Math.abs(optimizedState.speedMetersPerSecond) > 0) {
        setRotationPosition(optimizedState.angle);
        // }
        setDriveMotor(ControlMode.Velocity, optimizedState.speedMetersPerSecond);
    }

    public void setSwerveModuleState(SwerveModuleState state, boolean noteLastState) {
        if (Math.abs(state.speedMetersPerSecond) <= 0 && noteLastState) {
            state.angle = getModuleRotation();
        }

        setSwerveModuleState(state);
        // SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
        // getModuleRotation());
        // // if (Math.abs(optimizedState.speedMetersPerSecond) > 0) {
        // setRotationPosition(optimizedState.angle);
        // // }
        // setDriveMotor(ControlMode.Velocity, optimizedState.speedMetersPerSecond);
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