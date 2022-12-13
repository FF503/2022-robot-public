package org.frogforce503.robot2022.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.OI;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.commands.ClimbCommands.ClimbFirstBarSequential;
import org.frogforce503.robot2022.commands.ClimbCommands.ClimbSecondBarSequential;
import org.frogforce503.robot2022.commands.ClimbCommands.ClimbSecondBarSequentialSecond;
import org.frogforce503.robot2022.commands.ClimbCommands.ClimbThirdBarSequential;
import org.frogforce503.robot2022.commands.ClimbCommands.PrepForClimbSequential;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class Climber extends Subsystem {
    private static Climber instance = null;
    DigitalInput leftZeroSwitch, rightZeroSwitch, leftFixedSwitch, rightFixedSwitch;
    CANSparkMaxWrapper climbLeftMotor, climbRightMotor;
    DoubleSolenoid climberRotator;
    private ClimberStates curClimberState;
    private BarStates curBarState;
    private boolean readyToClimb = true;

    int lastLeftTarget = -1;
    int lastRightTarget = -1;

    int targetEncoderCountBoth = 0;
    int targetEncoderCountLeft = 0;
    int targetEncoderCountRight = 0;
    int startingEncoderValue = 0;

    double retractPower = -0.08;

    final double kInchesPerPulleyRev = 4.37; // FIXME: this is a placeholder, original 1.25 * Math.PI
    final double kEncoderCountsPerInch = (8.0) / kInchesPerPulleyRev;

    private final NetworkTable climberTable;
    private final ShuffleboardLayout climberList;

    Debouncer leftFixedOnBar = new Debouncer(0.2, Debouncer.DebounceType.kBoth);
    Debouncer rightFixedOnBar = new Debouncer(0.2, Debouncer.DebounceType.kBoth);

    public Climber() {
        climberTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Climber")
                .getSubTable("Climber");

        climberList = Shuffleboard.getTab("Climber").getLayout("Climber", BuiltInLayouts.kList)
                .withPosition(0, 0).withSize(3, 7);

        climberRotator = new DoubleSolenoid(Constants.PCMType,
                Robot.bot.climberForwardID, Robot.bot.climberReverseID);

        climbLeftMotor = new CANSparkMaxWrapper(Robot.bot.climbLeftMotorID, MotorType.kBrushless);
        configureClimbMotor(climbLeftMotor, true);
        climbRightMotor = new CANSparkMaxWrapper(Robot.bot.climbRightMotorID,
                MotorType.kBrushless);
        configureClimbMotor(climbRightMotor, false);

        leftZeroSwitch = new DigitalInput(Robot.bot.leftClimbZeroID);
        rightZeroSwitch = new DigitalInput(Robot.bot.rightClimbZeroID);
        leftFixedSwitch = new DigitalInput(Robot.bot.leftFixedID);
        rightFixedSwitch = new DigitalInput(Robot.bot.rightFixedID);

        curClimberState = ClimberStates.OFF;
        curBarState = BarStates.FLOOR;
    }

    public void configureClimbMotor(CANSparkMaxWrapper motor, boolean inverted) {
        motor.restoreFactoryDefaults();
        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);

        motor.setInverted(inverted);

        // SLOW
        motor.selectProfileSlot(0);
        motor.setP(0, 5e-5);
        motor.setI(0, 1e-6);
        motor.setIzone(0, 0.0);
        motor.setFF(0, 0.000156);
        motor.setCruiseVelocity(0, 3200.0);
        motor.setAcceleration(0, 3000.0);
        motor.setAllowableClosedLoopError(0, 0);

        // FAST
        motor.selectProfileSlot(1);
        motor.setP(1, 5e-5);
        motor.setI(1, 1e-6);
        motor.setIzone(1, 0);
        motor.setFF(1, 0.000156);
        motor.setCruiseVelocity(1, 5700.0);
        motor.setAcceleration(1, 10000.0);
        motor.setAllowableClosedLoopError(1, 0);

        // RETRACTING (onto bar)
        motor.selectProfileSlot(2);
        motor.setP(2, 5e-5);
        motor.setI(2, 1e-6);
        motor.setIzone(2, 0);
        motor.setFF(2, 0.0004);
        motor.setCruiseVelocity(2, 3600.0);
        motor.setAcceleration(2, 6500.0);
        motor.setAllowableClosedLoopError(2, 0);

        // motor.getPIDController().setFF(0, 0);
        // motor.setIzone(0, 0);

        motor.setPositionConversionFactor(1);

        motor.setEncoderPosition(startingEncoderValue);

        motor.setIdleMode(IdleMode.kBrake);
        // motor.setSmartCurrentLimit(60);
        motor.burnFlash();
    }

    /// Limit switches
    private boolean isLeftFixedOnBar() {
        return !leftFixedSwitch.get();
    }

    private boolean isRightFixedOnBar() {
        return !rightFixedSwitch.get();
    }

    public boolean isRightClimberZeroed() {
        return !rightZeroSwitch.get();
    }

    public boolean isLeftClimberZeroed() {
        return !leftZeroSwitch.get();
    }
    /// END Limit Switches

    /// Climber states
    public enum ClimberStates {
        FINISHED, ALIGNING, SLOW_RETRACTING, RETRACTING, EXTENDING, SLOW_EXTENDING, SHIFTING, TILTING, ROTATING_DOWN,
        ROTATING_UP,
        HOLDING, ZEROING, OFF,
        TELEOP
    }

    public ClimberStates getClimberState() {
        return curClimberState;
    }

    public void setClimberStates(ClimberStates s) {
        curClimberState = s;
    }

    // Bar states
    public enum BarStates {
        FLOOR, FLOOR_PREPPED, MID, MID_2, HIGH, TRAVERSAL
    }

    public BarStates getBarState() {
        return curBarState;
    }

    public void setBarStates(BarStates s) {
        curBarState = s;
    }

    public void setReadyToClimb(boolean hasFinishedStage) {
        readyToClimb = hasFinishedStage;
    }

    public boolean getReadyToClimb() {
        return readyToClimb;
    }
    ///

    /// State Control Methods

    public void moveClimber(double inches, boolean slowSetpoint) {
        targetEncoderCountBoth = (int) (inches * kEncoderCountsPerInch);
        if (targetEncoderCountBoth < climbLeftMotor.getEncoderPosition()) {
            if (slowSetpoint) {
                setClimberStates(ClimberStates.SLOW_RETRACTING);
            } else {
                setClimberStates(ClimberStates.RETRACTING);
            }
        } else {
            if (slowSetpoint) {
                setClimberStates(ClimberStates.SLOW_EXTENDING);
            } else {
                setClimberStates(ClimberStates.EXTENDING);
            }
        }
    }

    public void rezeroClimber() {
        setClimberStates(ClimberStates.ZEROING);
    }

    @Override
    public void stop() {
        setClimberStates(ClimberStates.OFF);
        climbRightMotor.stopMotor();
        climbLeftMotor.stopMotor();
    }

    // END State Control Methods

    /// Base Motor and Pneumatic Control Methods

    public void zeroEncoders() {
        climbRightMotor.setEncoderPosition(0.0);
        climbLeftMotor.setEncoderPosition(0.0);
    }

    public double getExtensionSetpoint() {
        return targetEncoderCountBoth / kEncoderCountsPerInch;
    }

    public void extendLeftDistance(double distance) {
        // FIXME send motors power based on distance
        targetEncoderCountLeft = (int) (distance * kEncoderCountsPerInch);
    }

    public void extendRightDistance(double distance) {
        targetEncoderCountRight = (int) (distance * kEncoderCountsPerInch);
    }

    public void rotateDown() { // Away from intake
        climberRotator.set(DoubleSolenoid.Value.kReverse);
    }

    public void rotateUp() { // Towards Intake
        climberRotator.set(DoubleSolenoid.Value.kForward);
    }

    public void toggleRotate() {
        if (climberRotator.get() == DoubleSolenoid.Value.kForward) {
            climberRotator.set(DoubleSolenoid.Value.kReverse);
        } else {
            climberRotator.set(DoubleSolenoid.Value.kForward);
        }
    }
    /// END Motor and Pneumatic Control Methods

    // Automation Checks

    public boolean isFixedOnBar() {
        return leftFixedOnBar.calculate(isLeftFixedOnBar()) && rightFixedOnBar.calculate(isRightFixedOnBar());
    }

    /**
     * Helps to avoid application of torque to robot's swing.
     * 
     * @return If robot pitch is within a +- 10 degree tolerance of neutral.
     */
    public boolean pitchWithinRange() {
        return Math.abs(Constants.kNeutralPitch - Swerve.getInstance().getPitch()) < 10.0;
    }

    // END Automation Checks

    public void toggleTeleOpControl() {
        if (getClimberState() != ClimberStates.TELEOP) {
            setClimberStates(ClimberStates.TELEOP);
        } else {
            setClimberStates(ClimberStates.OFF);
        }
    }

    /**
     * Manual Left-Right Operator Control
     * Up on joystick extends, down on joystick retracts. Max output of 50% Power
     */
    // @formatter
    public void joystickControl() {
        double rightJoyStickValue;
        double leftJoyStickValue;

        if (StateEngine.getInstance().getClimbingSwap()) {
            rightJoyStickValue = -OI.getDriverRightYValue() * 0.83;
            leftJoyStickValue = -OI.getDriverLeftYValue() * 0.83;
        } else {
            rightJoyStickValue = -OI.getOperatorRightYValue() * .83;
            leftJoyStickValue = -OI.getOperatorLeftYValue() * .83;
        }

        double rightJoystick = rightJoyStickValue;
        double leftJoystick = leftJoyStickValue;

        if (!isRightClimberZeroed()) {
            if (rightJoystick > 0.1) {
                climbRightMotor.set(rightJoystick);
            } else if (rightJoystick < -0.1) {
                climbRightMotor.set(rightJoystick);
            } else {
                climbRightMotor.set(0);
            }
        } else if (isRightClimberZeroed()) {
            if (rightJoystick > 0.1) {
                climbRightMotor.set(rightJoystick);
            } else {
                climbRightMotor.set(0);
            }
        }

        if (!isLeftClimberZeroed()) {
            if (leftJoystick > 0.1) {
                climbLeftMotor.set(leftJoystick);
            } else if (leftJoystick < -0.1) {
                climbLeftMotor.set(leftJoystick);
            } else {
                climbLeftMotor.set(0);
            }
        } else if (isLeftClimberZeroed()) {
            if (leftJoystick > 0.1) {
                climbLeftMotor.set(leftJoystick);
            } else {
                climbLeftMotor.set(0);
            }
        }

    }

    public void initTelemetry() {
        climberList.addBoolean("Climber Telemetry Toggle", () -> outputtingTelemetry())
                .withWidget(BuiltInWidgets.kToggleSwitch);

        climberList.addBoolean("Climber Telemetry", () -> outputtingTelemetry());

        climberList.addString("Climber State", () -> getClimberState().toString());
    }

    @Override
    public void outputTelemetry() {

        climberTable.getEntry("Climb Bar state").setString(getBarState().toString());
        if (outputtingTelemetry()) {
            // climberTable.getEntry("PNEUMATIC
            // POSITION").setString(climberRotator.get().toString());
            climberTable.getEntry("Climber Left Switch").setBoolean(isLeftClimberZeroed());
            climberTable.getEntry("Climber RIght Switch").setBoolean(isRightClimberZeroed());
            climberTable.getEntry("Fixed Left Switch").setBoolean(isLeftFixedOnBar());
            climberTable.getEntry("Fixed Right Switch").setBoolean(isRightFixedOnBar());
            climberTable.getEntry("Target Position").setDouble(targetEncoderCountBoth / kEncoderCountsPerInch);
            climberTable.getEntry("Climb Motor Current").setDouble(climbLeftMotor.getOutputCurrent());

            // climberTable.getEntry("Climb Motor %
            // Output").setDouble(climbLeftMotor.getAppliedOutput());
            // climberTable.getEntry("Climb Motor % Output").setDouble(climbLeftMotor.());
            climberTable.getEntry("Robot Pitch").setDouble(Swerve.getInstance().getPitch());
            climberTable.getEntry("Climb Ready").setBoolean(getReadyToClimb());
            climberTable.getEntry("PID Slot")
                    .setNumber(climbLeftMotor.getSelectedProfileSlot());

            climberTable.getEntry("Climber Left Encoder Position")
                    .setNumber(climbLeftMotor.getEncoderPosition());
            climberTable.getEntry("Climber Right Encoder Position")
                    .setNumber(climbRightMotor.getEncoderPosition());
            climberTable.getEntry("Climber Left Motor Position")
                    .setNumber(climbLeftMotor.getEncoderPosition() / kEncoderCountsPerInch);
            // climberTable.getEntry("Climber Right Motor Position")
            // .setNumber(climbRightMotor.getEncoderPosition() / kEncoderCountsPerInch);
        }
        // Climber.getEntry("Climber right
        // switch").setBoolean(!extendRightSwitch.get());
        // SmartDashboard.putNumber("Climber right motor position",
        // climbRightMotor.getEncoderPosition() / kEncoderCountsPerInch);
        // SmartDashboard.putString("Climber State", curClimberState.name());
        // SmartDashboard.putBoolean("Climber right on target",
        // rightController.atSetpoint());
        // SmartDashboard.putNumber("Climber right target", targetEncoderCountRight);
        // SmartDashboard.putNumber("Climber right lastTarget", lastRightTarget);
        // SmartDashboard.putNumber("Climber right power", rightPower);
        // SmartDashboard.putBoolean("Climber right switch ", !extendRightSwitch.get());

    }

    public void updatePIDValues(CANSparkMaxWrapper motor) {
        motor.selectProfileSlot(0);
        motor.setP(0, climberTable.getEntry("Climber extP").getDouble(0.0));
        motor.setD(0, climberTable.getEntry("Climber extD").getDouble(0.0));
        motor.selectProfileSlot(1);
        motor.setP(1, climberTable.getEntry("Climber retP").getDouble(0.0));
        motor.setD(1, climberTable.getEntry("Climber retD").getDouble(0.0));
    }

    private boolean outputtingTelemetry() {
        return climberTable.getEntry("Climber Telemetry Toggle").getBoolean(false);
    }

    /// Retraction and Extension Handlers

    private boolean encoderNotWithinTargetRange(CANSparkMaxWrapper motor) {
        return (Math.abs(motor.getEncoderPosition() - targetEncoderCountBoth) > 2);
    }

    public boolean climbEncodersAtSetpoint() {
        return (!encoderNotWithinTargetRange(climbLeftMotor) && !encoderNotWithinTargetRange(climbRightMotor));
    }

    public boolean isClimberZeroed() {
        return !leftZeroSwitch.get() && !rightZeroSwitch.get();
    }

    public void setRetractionSetpoint(CANSparkMaxWrapper motor, int slot) {
        motor.selectProfileSlot(slot);
        motor.set(ControlMode.SmartMotion, targetEncoderCountBoth);
    }

    public void setExtensionSetpoint(CANSparkMaxWrapper motor, int slot) {
        motor.selectProfileSlot(slot);
        motor.set(ControlMode.SmartMotion, targetEncoderCountBoth);
    }
    /// END Cleanup Methods

    public void writePeriodicOutputs() {

        switch (curClimberState) {
            case OFF:
                climbLeftMotor.set(ControlMode.PercentOutput, 0);
                climbRightMotor.set(ControlMode.PercentOutput, 0);
                break;

            case TELEOP:
                joystickControl();
                break;

            case SLOW_EXTENDING:
                climbLeftMotor.selectProfileSlot(0);
                climbLeftMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);

                climbRightMotor.selectProfileSlot(0);
                climbRightMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);
                break;

            case EXTENDING:
                climbLeftMotor.selectProfileSlot(1);
                climbLeftMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);

                climbRightMotor.selectProfileSlot(1);
                climbRightMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);
                break;
            case SLOW_RETRACTING:

                climbLeftMotor.selectProfileSlot(0);
                climbRightMotor.selectProfileSlot(0);

                if (!isLeftClimberZeroed()) {
                    climbLeftMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);
                } else {
                    climbLeftMotor.set(ControlMode.PercentOutput, 0);
                }

                if (!isRightClimberZeroed()) {
                    climbRightMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);
                } else {
                    climbRightMotor.set(ControlMode.PercentOutput, 0);
                }
                break;

            case RETRACTING:

                climbLeftMotor.selectProfileSlot(2);
                climbRightMotor.selectProfileSlot(2);
                // insert once values and pitch range is collected
                // if (pitchWithinRange()) {
                // // full speed profile
                // climbLeftMotor.selectProfileSlot(2);
                // climbRightMotor.selectProfileSlot(2);
                // } else {
                // // slow retraction speed profile
                // climbLeftMotor.selectProfileSlot(0);
                // climbRightMotor.selectProfileSlot(0);
                // }

                if (!isLeftClimberZeroed()) {
                    climbLeftMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);
                } else {
                    climbLeftMotor.set(ControlMode.PercentOutput, 0);
                }

                if (!isRightClimberZeroed()) {
                    climbRightMotor.set(ControlMode.SmartMotion, targetEncoderCountBoth * 1.0);
                } else {
                    climbRightMotor.set(ControlMode.PercentOutput, 0);
                }
                break;

            case FINISHED:
                break;

            case ZEROING:
                if (!isRightClimberZeroed()) {
                    climbRightMotor.set(retractPower);
                } else {
                    climbRightMotor.setEncoderPosition(0.0);
                    climbRightMotor.set(-0.02);
                }

                if (!isLeftClimberZeroed()) {
                    climbLeftMotor.set(retractPower);
                } else {
                    climbLeftMotor.setEncoderPosition(0.0);
                    climbLeftMotor.set(-0.02);
                }

                if (isLeftClimberZeroed() && isRightClimberZeroed()) {
                    curClimberState = ClimberStates.OFF;
                }
                break;

            default:
                curClimberState = ClimberStates.OFF;
                break;
        }

    }

    public void handleClimberStates() {
        System.out.println("scheduled climb command");

        boolean ready = getReadyToClimb();
        if (ready) {
            Climber.getInstance().setReadyToClimb(false);
            DriverFeedback.getInstance().setLEDState(LEDStates.RED);
            switch (Climber.getInstance().getBarState()) {
                case FLOOR:
                    // System.out.println("ON FLOOR");
                    new PrepForClimbSequential().schedule();
                    Climber.getInstance().setBarStates(BarStates.FLOOR_PREPPED);
                    break;
                case FLOOR_PREPPED:
                    new ClimbFirstBarSequential().schedule();
                    Climber.getInstance().setBarStates(BarStates.MID);
                    break;
                case MID:
                    new ClimbSecondBarSequential().schedule();
                    Climber.getInstance().setBarStates(BarStates.MID_2);
                    break;
                case MID_2:
                    new ClimbSecondBarSequentialSecond().schedule();
                    Climber.getInstance().setBarStates(BarStates.HIGH);
                    break;
                case HIGH:
                    new ClimbThirdBarSequential().schedule();
                    Climber.getInstance().setBarStates(BarStates.TRAVERSAL);
                    break;
                case TRAVERSAL:
                    break;
            }
        }
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
        stop();

    }

    public void readPeriodicInputs() {

    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }
}