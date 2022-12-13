package org.frogforce503.robot2022.subsystems;

import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.JudgeZone.BallColors;
import org.frogforce503.robot2022.subsystems.vision.LimelightProcessor;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class DriverFeedback extends Subsystem {
    // Put Lists & NetworkTables when needed

    static DriverFeedback instance = null;

    private final NetworkTable driverTable;
    private final ShuffleboardLayout driverTableList, operatorTableList, technicianTableList;
    private static Spark m_blinkin = null;

    private LEDStates currentLEDState = LEDStates.OFF;

    public LEDStates getLEDStates() {
        return currentLEDState;
    }

    public void setLEDState(LEDStates s) {
        currentLEDState = LEDStates.OFF;
    }

    public static DriverFeedback getInstance() {
        return instance == null ? instance = new DriverFeedback() : instance;
    }

    private ShuffleboardComponent driverCameraStream = null;
    private ShuffleboardComponent limelightCameraStream = null;
    private ShuffleboardComponent LEDColor;

    public DriverFeedback() {
        driverTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("DriverStation");
        driverTableList = Shuffleboard.getTab("DriverStation").getLayout("Driver FeedBack Booleans",
                BuiltInLayouts.kList).withPosition(12, 0).withSize(2, 4);
        operatorTableList = Shuffleboard.getTab("DriverStation").getLayout("Operator FeedBack Booleans",
                BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 4);
        technicianTableList = Shuffleboard.getTab("Technician").getLayout("Technician Pre-match Checks",
                BuiltInLayouts.kList).withPosition(0, 0).withSize(3, 4);

        if (RobotBase.isReal()) {
            CameraServer.startAutomaticCapture();

            driverCameraStream = Shuffleboard.getTab("DriverStation").addCamera("DriverCamera Stream", "USB Camera",
                    Robot.bot.usbCameraURL);
        }

        m_blinkin = new Spark(Robot.bot.LED_ID);
        m_blinkin.setSafetyEnabled(false);
    }

    private double getLEDColor() {
        return getLEDStates().getColor();
    }

    public enum LEDStates {
        // values acquired from:
        // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf (pg 14-15)
        FLASH(0.07), BLUE(0.87), DARK_GREEN(0.75), OFF(0.99), RED(0.61), YELLOW(0.69), RAINBOW(-0.99),
        HOT_PINK(0.57), VIOLET(0.91);

        private double value;

        LEDStates(double value) {
            this.value = value;
        }

        public double getColor() {
            return this.value;
        }

    }

    public void updateIntakingColors() {
        if (JudgeZone.getInstance().getColor() == BallColors.BLUE) {
            DriverFeedback.getInstance().setLEDState(LEDStates.BLUE);
        } else if (JudgeZone.getInstance().getColor() == BallColors.RED) {
            DriverFeedback.getInstance().setLEDState(LEDStates.RED);
        } else if (JudgeZone.getInstance().getColor() == BallColors.NONE) {
            DriverFeedback.getInstance().setLEDState(LEDStates.YELLOW);
            if (Turret.getInstance().isTurretOnTarget() && LimelightProcessor.getInstance().isTargetVisible()) {
                DriverFeedback.getInstance().setLEDState(LEDStates.HOT_PINK);
            }
        }

    }

    public void initTelemetry() {
        if (RobotBase.isReal())
            driverCameraStream.withPosition(6, 0).withSize(5, 5);

        technicianTableList.addBoolean("Limelight Target Visible",
                () -> LimelightProcessor.getInstance().isTargetVisible());
        technicianTableList.addNumber("Limelight TX", () -> LimelightProcessor.getInstance().getTX());

        driverTableList.addBoolean("Turret On Target", () -> Turret.getInstance().isTurretOnTarget());
        driverTableList.addBoolean("Preset Override", () -> StateEngine.getInstance().isPresetOverride());
        driverTableList.addBoolean("Persistent Override", () -> StateEngine.getInstance().isPersistPreset());

        operatorTableList.addBoolean("Lower Beam Break", () -> JudgeZone.getInstance().hasBall());
        operatorTableList.addBoolean("Tower Beam Break", () -> Tower.getInstance().hasBall());
        Shuffleboard.getTab("GameSpec")
                .addString("Robot State", () -> StateEngine.getInstance().getRobotState().toString())
                .withPosition(0, 5);
        operatorTableList.addString("Robot State", () -> StateEngine.getInstance().getRobotState().toString());

        operatorTableList.addNumber("MATCH TIME", () -> (int) Timer.getMatchTime());
        Shuffleboard.getTab("DriverStation").addNumber("Shot Ball Count", Tower.getInstance()::getShotCount)
                .withPosition(0, 5);
    }

    public void outputTelemetry() {
    }

    public void writePeriodicOutputs() {
        set(currentLEDState.getColor());
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    public void set(double val) {
        if ((val >= -1.0) && (val <= 1.0)) {
            m_blinkin.set(val);
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
        // TODO Auto-generated method stub

    }

}