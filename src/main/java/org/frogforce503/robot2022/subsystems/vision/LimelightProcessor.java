package org.frogforce503.robot2022.subsystems.vision;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.RobotState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightProcessor {

    private static LimelightProcessor instance = null;

    private final NetworkTable table;
    private final NetworkTableEntry ledMode, pipeline, camMode, stream, ct;
    private final List<NetworkTableEntry> combinedTarget;
    private final LinkedList<VisionData> collectedData = new LinkedList<VisionData>();
    private double tx, ty, ta, dist;

    private boolean currentlyMagnified = false;
    private boolean forceMagnified = false;
    // private LimelightState currentState = LimelightState.UP;
    // private TargetingState curTargetingState = TargetingState.OFF, previousState
    // = TargetingState.OFF;
    // private Pipeline currentPipeline;

    private DoubleSolenoid limelightSolenoid;

    private LinearFilter txFilter = LinearFilter.singlePoleIIR(0.05, 0.01);
    private LinearFilter tyFilter = LinearFilter.singlePoleIIR(0.05, 0.01);

    private Debouncer risingPipelineDebouncer = new Debouncer(.6, DebounceType.kRising);
    private Debouncer fallingPipelineDebouncer = new Debouncer(.5, DebounceType.kRising);
    private Debouncer notVisibleDebouncer = new Debouncer(.5, DebounceType.kBoth);

    /**
     * @param targetRange Target range, in inches
     */
    private double currentTargetRange;

    private LimelightProcessor() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        ledMode = table.getEntry("ledMode");
        pipeline = table.getEntry("pipeline");
        camMode = table.getEntry("camMode");
        ct = table.getEntry("camtran");
        stream = table.getEntry("stream");

        combinedTarget = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"), table.getEntry("ta"),
                table.getEntry("tv"));

        setPipeline(Pipeline.DRIVER);
    }

    public static LimelightProcessor getInstance() {
        return instance == null ? instance = new LimelightProcessor() : instance;
    }

    public void stopTargeting() {
        setPipeline(Pipeline.DRIVER);
        this.tx = 0;
        this.ty = 0;
        this.ta = 0;
        this.dist = 0;
        SmartDashboard.putNumber("distance", 0);
        // Swerve.getInstance().killVisionTranslationInput();
        // Swerve.getInstance().killVisionAngularInput();
    }

    public void refreshStandard(Pipeline pipeline) {
        // setPipeline(pipeline.magnifiedID);
        this.tx = txFilter.calculate(getTX());
        this.ty = tyFilter.calculate(getTY());
        this.ta = getTA();
        SmartDashboard.putNumber("Limelight CURRENTLY MAGNIFIED", this.currentlyMagnified ? 100 : 0);
        if (!this.currentlyMagnified) {
            this.dist = distFromTarget(0.0);
        } else {
            this.dist = distFromTarget(Robot.bot.limelightMagnifiedOffset);
            // this.dist = distFromTarget(0.0);
        }

        if (!forceMagnified) {
            notVisibleDebouncer.calculate(!isTargetVisible());
            if (isTargetVisible() && pipeline == Pipeline.HUB) {
                if (risingPipelineDebouncer.calculate(this.dist > 210)) { // ADD debounced
                    // setPipeline(pipeline.magnifiedID);
                    // this.currentlyMagnified = true;
                } else if (fallingPipelineDebouncer.calculate(this.dist < 190)) {
                    setPipeline(pipeline.standardID);
                    this.currentlyMagnified = false;
                }
            } else if (notVisibleDebouncer.calculate(!isTargetVisible())) {
                setPipeline(pipeline.standardID);
                this.currentlyMagnified = false;
            }
        } else {
            setPipeline(pipeline.magnifiedID);
            this.currentlyMagnified = true;
        }
        SmartDashboard.putNumber("distance", this.dist);
    }

    public void blink() {
        if (ledMode.getDouble(0) != 2) {
            ledMode.setNumber(2);
        }
    }

    public void ledOn(final boolean on) {
        if (ledMode.getDouble(1) != 0 && on) {
            ledMode.setNumber(0);
        } else if (ledMode.getDouble(0) != 1 && !on) {
            ledMode.setNumber(1);
        }
    }

    public void setDriverMode() {
        camMode.setNumber(1);
    }

    public void setVisionMode() {
        camMode.setNumber(0);
    }

    public void setStreamMode(final int id) {
        stream.setNumber(id);
    }

    /**
     * Private accessor to networktables
     *
     * @param id Pipeline ID
     */
    private void setPipeline(final int id) {
        pipeline.setNumber(id);
    }

    public void setPipeline(final Pipeline p) {
        // this.currentPipeline = p;
        setPipeline(p.standardID);
    }

    public void setPipeline(final Pipeline p, boolean magnify) {
        // this.currentPipeline = p;
        setPipeline(magnify ? p.magnifiedID : p.standardID);
    }

    public void forceMagnified(boolean magnified) {
        this.forceMagnified = magnified;
    }

    public double getPipelineNumber() {
        return pipeline.getDouble(0.0);
    }

    public double getTX() {
        return combinedTarget.get(0).getDouble(0.0);
    }

    public double getTY() {
        return combinedTarget.get(1).getDouble(0.0);
    }

    public double getTA() {
        return combinedTarget.get(2).getDouble(0.0);
    }

    public double getTV() {
        return (combinedTarget.get(3).getDouble(0.0));
    }

    public double getHubError() {
        refreshStandard(Pipeline.HUB);
        return tx;
    }

    public double[] getTranslation() {
        return ct.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
    }

    public boolean isTargetVisible() {
        return getTV() == 1.0;
    }

    public void magnifyCurrentPipeline() {
        setPipeline(0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculation routines
    //////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Calculates the horizontal straight-line from the camera to the outer
     * goal
     *
     * @return distance, in inches
     */

    public double getDistFromHub() {
        return this.dist;
    }

    private double distFromTarget(double angularOffset) {
        double distance = (Constants.HEIGHT_OF_HUB - Robot.bot.limelightFloorClearance)
                / Math.tan(Math.toRadians(ty + angularOffset + Robot.bot.limelightVerticalAngle));
        SmartDashboard.putNumber("distance from target", distance);
        return distance;
    }

    public double distFromOuterTargetX() {
        return Math.sin(Math.toRadians(tx + RobotState.getInstance().getCurrentTheta())) * dist;
    }

    public double distFromOuterTargetY() {
        return Math.cos(Math.toRadians(tx + RobotState.getInstance().getCurrentTheta())) * dist;
    }

    public boolean getDistanceOnTarget() {
        return (Math.abs(currentTargetRange - dist) < Robot.bot.visionTargetingRangeTolerance);
    }

    public Translation2d vectorToOuterTarget() {
        return new Translation2d(distFromOuterTargetX(), distFromOuterTargetY());
    }

    public boolean onTarget() {
        return Math.abs(tx) < Robot.bot.limelightTXTolerance;
    }

    void updateData(final double x, final double y, final double angle, final double[] ct) {
        updateData(new VisionData(x, y, angle, isTargetVisible(), ct));
    }

    private void updateData(final VisionData data) {
        if (collectedData.size() >= 15) {
            collectedData.removeFirst();
        }

        collectedData.add(data);
    }

    public enum Pipeline {
        DRIVER(0), HUB(1, 2);

        int standardID, magnifiedID;

        Pipeline(final int standardID) {
            this(standardID, standardID);
        }

        Pipeline(final int standardID, final int magnifiedID) {
            this.standardID = standardID;
            this.magnifiedID = magnifiedID;
        }
    }

    private class VisionData {
        public final double x, y, angle;
        public final boolean seesTarget;
        public final double[] camtran;

        VisionData(final double x, final double y, final double angle, final boolean seesTarget,
                final double[] camtran) {
            this.x = x;
            this.y = y;
            this.angle = angle;
            this.seesTarget = seesTarget;
            this.camtran = camtran;
        }

        @Override
        public String toString() {
            return "x: " + x + "y: " + y + "angle: " + angle + "sees: " + seesTarget + "Trans"
                    + Arrays.toString(camtran);
        }
    }
}