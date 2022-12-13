package org.frogforce503.robot2022.subsystems;

import java.util.Map;

import org.frogforce503.lib.util.InterpolatingDouble;
import org.frogforce503.lib.util.InterpolatingTreeMap;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.vision.LimelightProcessor;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

// WE ARE USING THE WCP-0408 L16-100-35-6-R
// - 100mm extension distance, 35:1 gear ratio, 32mm/s speed 

public class Hood {
    private static Hood instance = null;
    private static Servo leftLinearActuator, rightLinearActuator;

    private static final double MAX_ANGLE = Robot.bot.kMaxHoodAngle;
    private static final double MIN_ANGLE = Robot.bot.kMinHoodAngle;
    private static final double kHoodSpeed = Robot.bot.kHoodSpeed;
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToHoodSetpoint = new InterpolatingTreeMap<>();

    private final NetworkTable hoodTable;

    private final NetworkTable shotmapHoodTable, shotmapOutTable;
    private final ShuffleboardLayout shotmapHoodList;

    public enum HoodStates {
        VISION, BATTER, MANUAL, OFF
    }

    private HoodStates curHoodState = HoodStates.OFF;

    private double hoodSetpoint;
    private double estimatedPos = Robot.bot.kMinHoodAngle - 10.0;

    public void populateTreeMapForHood() {
        for (int i = 0; i < Robot.bot.shotMapValues.length; i++) {
            kDistanceToHoodSetpoint.put(new InterpolatingDouble(Robot.bot.shotMapValues[i][0]),
                    new InterpolatingDouble(Robot.bot.shotMapValues[i][2]));
        }
    }

    private Hood() {
        populateTreeMapForHood();
        hoodTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Hood");
        shotmapOutTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Shotmap")
                .getSubTable("mapOut");
        shotmapHoodTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Shotmap")
                .getSubTable("Hood");
        shotmapHoodList = Shuffleboard.getTab("Shotmap").getLayout("Hood", BuiltInLayouts.kList);

        leftLinearActuator = new Servo(Robot.bot.leftLinearActuatorID);
        leftLinearActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightLinearActuator = new Servo(Robot.bot.rightLinearActuatorID);
        rightLinearActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        setHoodAngle(Robot.bot.kMinHoodAngle);
    }

    public static Hood getInstance() {
        return instance == null ? instance = new Hood() : instance;
    }

    public void setHoodState(HoodStates s) {
        curHoodState = s;
    }

    /**
     * Write Periodic Hood Angle Outputs
     */
    public void writePeriodicOutputs() {
        updateCurPos();
        if (shotmapTuningActive()) {
            setManualHoodSetpoint(shotmapHoodSetpoint());
        }
        switch (curHoodState) {
            case OFF:
                setHoodAngle(Robot.bot.kMinHoodAngle);
                break;
            case VISION:
                if (StateEngine.getInstance().getRobotState() != RobotStates.SHOOTING) {
                    hoodSetpoint = getHoodSetpointForDistance(LimelightProcessor.getInstance().getDistFromHub());
                }
                setHoodAngle(hoodSetpoint);
                break;
            case BATTER:
                setHoodAngle(Robot.bot.kBatterHoodAngle);
                break;
            case MANUAL:
                setHoodAngle(hoodSetpoint);
                break;
        }
    }

    public void setManualHoodSetpoint(double setpoint) {
        setHoodState(HoodStates.MANUAL);
        setHoodAngle(setpoint);
    }

    public void setHoodAngle(double angle) {
        setServoAngle(angle, leftLinearActuator);
        setServoAngle(angle, rightLinearActuator);
        hoodSetpoint = angle;
    }

    private void setServoAngle(double angle, Servo servo) {
        hoodSetpoint = Math.max(Math.min(angle, MAX_ANGLE), MIN_ANGLE);
        hoodSetpoint *= (100.0 / 180.0); // convert to mm
        servo.setSpeed((hoodSetpoint / 100 * 2) - 1);
        // System.out.println("HOOD ANGLE GET = " + servo.getAngle());
    }

    private double getAngle(Servo servo) {
        double angle = servo.getAngle();
        return angle;
    }

    double lastTime = 0;

    /**
     * Run this method in any periodic function to update the position estimation of
     * the servo
     */
    public void updateCurPos() {
        double curTime = Timer.getFPGATimestamp();
        double dt = curTime - lastTime;
        if (estimatedPos > hoodSetpoint + kHoodSpeed * dt) {
            estimatedPos -= kHoodSpeed * dt;
        } else if (estimatedPos < hoodSetpoint - kHoodSpeed * dt) {
            estimatedPos += kHoodSpeed * dt;
        } else {
            estimatedPos = hoodSetpoint;
        }
        lastTime = curTime;
    }

    /**
     * @return true if hood is within 2 "degrees" of setpoint
     */
    public boolean onTarget() {
        double delta = Math.abs(hoodSetpoint - estimatedPos);
        return delta < 2;
    }

    // FIXME:Make sure min max values are correct - Max Distance setpoint
    public double getHoodSetpointForDistance(double distance) {
        return kDistanceToHoodSetpoint
                .getInterpolated(new InterpolatingDouble(Math.max(Math.min(distance, 300.0), 0.0))).value;
    }

    public double getHoodSetpoint() {
        return hoodSetpoint;
    }

    public void initTelemetry() {
        Shuffleboard.getTab("GameSpec").getLayout("Hood", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 5);

        Shuffleboard.getTab("GameSpec").getLayout("Hood", BuiltInLayouts.kList)
                .addBoolean("Hood Telemetry Toggle",
                        () -> outputtingTelemetry())
                .withSize(2, 1).withWidget(BuiltInWidgets.kToggleSwitch);

        Shuffleboard.getTab("GameSpec").getLayout("Hood", BuiltInLayouts.kList)
                .addBoolean("Hood Telemetry",
                        () -> outputtingTelemetry())
                .withSize(2, 1);

        Shuffleboard.getTab("GameSpec").getLayout("Hood", BuiltInLayouts.kList)
                .addString("Hood State", () -> curHoodState.name());

        shotmapHoodList.withPosition(6, 0).withSize(3, 5);
        shotmapHoodList.addNumber("Shotmap Hood Setpoint", () -> (Math.round(shotmapHoodSetpoint() / 1.0) * 1))
                .withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", Robot.bot.kMinHoodAngle, "max", Robot.bot.kMaxHoodAngle));

    }

    public void outputTelemetry() {
        if (outputtingTelemetry()) {
            hoodTable.getEntry("Hood Setpoint").setNumber(hoodSetpoint);
            hoodTable.getEntry("Estimated Hood Position").setNumber(estimatedPos);
            hoodTable.getEntry("Hood On Target").setBoolean(onTarget());
        }

    }

    private boolean outputtingTelemetry() {
        return hoodTable.getEntry("Hood Telemetry Toggle").getBoolean(false);
    }

    private double shotmapHoodSetpoint() {
        return shotmapHoodTable.getEntry("Shotmap Hood Setpoint").getDouble(MIN_ANGLE);
    }

    private boolean shotmapTuningActive() {
        return shotmapOutTable.getEntry("Shotmap Tuning Toggle").getBoolean(false);
    }
}