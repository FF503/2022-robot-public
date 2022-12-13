package org.frogforce503.robot2022.commands;

import java.util.function.Supplier;

// import org.frogforce503.lib.follower.CargoFetcher;
import org.frogforce503.lib.follower.MarkerConfig;
import org.frogforce503.lib.follower.Path;
import org.frogforce503.lib.follower.SwervePathFollower;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.SwerveControlState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NewSwerveFollowPathCommand extends CommandBase {
    Path path;
    String filename;
    double endTime;
    // Timer timer;
    // Timer waitTimer;
    int totalIterations;
    int currentIteration = 1;
    double startTime = 0;
    double waitStart = 0;
    double timeSpentWaiting = 0;
    double timeOffset = 0;
    boolean isWaiting = false;
    boolean shouldResetOdometry = true;
    boolean ignoreEndAngle = false;

    Timer pathTimer;
    Timer waitTimer;

    Pose2d startingState = new Pose2d();

    // double waitStartTime = 0;
    // int waitIterations = 0;
    // double waitBeginTime = 0;
    // boolean begunWaiting = false;
    // boolean waitCondition = false;

    // TODO: Make a way for the user of SwerveFollowPathCommand to call the
    public NewSwerveFollowPathCommand(String pathFile) {
        this(pathFile, true);
        // path = Path.fromCSV(pathFile);
        // filename = pathFile;
        // SmartDashboard.putNumber(pathFile + " Path Length", path.getStates().size());
        // endTime = path.getTotalTimeSeconds();
        // totalIterations = path.getStates().size();
        // timer = new Timer();
        // waitTimer = new Timer();
    }

    public NewSwerveFollowPathCommand(String pathFile, boolean shouldResetOdometry) {
        path = Path.fromCSV(pathFile);
        filename = pathFile;
        SmartDashboard.putNumber(pathFile + " Path Length", path.getStates().size());
        endTime = path.getTotalTimeSeconds();
        totalIterations = path.getStates().size();
        this.shouldResetOdometry = shouldResetOdometry;
        this.ignoreEndAngle = false;
        // timer = new Timer();
        // waitTimer = new Timer();
    }

    public void recreatePath() {
        path = Path.fromCSV(filename);
    }

    public Pose2d end() {
        return path.getStates().get(path.getStates().size() - 1).poseMeters;
    }

    public Path getPath() {
        return path;
    }

    public void saveStartingState() {
        startingState = Swerve.getInstance().getPoseMeters();
    }

    public void ignoreEndAngle() {
        ignoreEndAngle = true;
    }

    public void refreshStartingState() {
        // FIXME: make this work (if it is necessary)

        // Swerve.getInstance().setAngle((180 - Swerve.getInstance().getAngleDegrees())
        // % 360);
        // Swerve.getInstance()
        // .setAngle(startingState.getRotation().getDegrees() +
        // Swerve.getInstance().getAngleDegrees());
        // TODO: Adjust pose estimate based on red or blue alliance (mirror if
        // necessary)
    }

    public NewSwerveFollowPathCommand setWaitCondition(Supplier<Boolean> condition) {
        this.path.setWaitCondition(condition);
        return this;
    }

    public NewSwerveFollowPathCommand transformBy(Transform2d transform) {
        path = path.transformBy(transform);
        return this;
    }

    @SafeVarargs
    public final NewSwerveFollowPathCommand setWaitConditions(Supplier<Boolean>... condition) {
        NewSwerveFollowPathCommand cmd = this;

        for (Supplier<Boolean> c : condition) {
            cmd = cmd.setWaitCondition(c);
        }

        return cmd;
    }

    public NewSwerveFollowPathCommand configureMarkers(MarkerConfig config) {
        this.path.configureMarkers(config);
        return this;
    }

    // example for using wait condition with states.
    // public SwerveFollowPathCommand setWaitCondition(GameSpecState state) {
    // this.path.setWaitCondition(() -> GameSpec.getInstance().isAt(state));
    // }

    @Override
    public void initialize() {
        // if (this.shouldResetOdometry)
        // Swerve.getInstance().resetOdometry(path.getStates().get(0).poseMeters);

        Swerve.getInstance().drawPath(path);
        Swerve.getInstance().drawMarkers(path.getMarkers());
        Swerve.getInstance().startPlottingRobotPath();
        // CargoFetcher.getInstance().setPathpoints(path.getStates());
        startTime = Timer.getFPGATimestamp();
        pathTimer = new Timer();
        pathTimer.start();
        waitTimer = new Timer();
        timeOffset = 0;
        DriverFeedback.getInstance().setLEDState(LEDStates.BLUE);
        // startTime = timer.get();
        // timer.start();
    }

    @Override
    public void execute() {
        double currentTime = pathTimer.get() + timeOffset;
        Path.State currentTarget = path.sample(currentTime);

        SmartDashboard.putNumber("X Setpoint", currentTarget.poseMeters.getX());
        SmartDashboard.putNumber("Y Setpoint", currentTarget.poseMeters.getY());
        SmartDashboard.putNumber("Rotation From End", currentTarget.poseMeters.getRotation().getDegrees());

        // System.out.println(currentTarget.poseMeters.toString());

        path.checkMarkers(Swerve.getInstance().getPoseMeters());

        if (currentTarget.type() == "Wait") {
            if (!isWaiting) {
                pathTimer.stop();
                waitTimer.reset();
                waitTimer.start();
                isWaiting = true;
                System.out.println("STARTING TO WAIT");
            }

            boolean exit = false;
            boolean isTimeWait = false;
            Path.Wait targetAsWait = (Path.Wait) currentTarget;
            System.out.println(targetAsWait.getWaitType().name());
            isTimeWait = targetAsWait.getWaitType() == Path.Wait.WaitType.TIME;
            exit = isTimeWait
                    ? waitTimer.get() > targetAsWait.getWaitTime()
                    : targetAsWait.isSatisfied();

            if (exit) {
                pathTimer.start();
                waitTimer.stop();
                System.out.println("WAIT HAS FINISHED AFTER " + waitTimer.get());

                if (isTimeWait)
                    timeOffset += 0.02;

                isWaiting = false;
            }

            Swerve.getInstance().setPathFollowerSpeeds(Swerve.ZERO_CHASSIS_SPEED);
            System.out.println("WAITITNGGG");

            // if (!isWaiting) {
            // waitStart = Timer.getFPGATimestamp();
            // isWaiting = true;
            // }

            // if (CargoFetcher.getInstance().isCurrentlySearching()) {
            // // System.out.println("First If");
            // if (CargoFetcher.getInstance().isHandlingUnconventionalSearchZone())
            // Swerve.getInstance().stop();
            // return;
            // }

            // boolean waitFinished = false;

            // if (CargoFetcher.getInstance().justFinishedSearching()) {
            // waitFinished = true;
            // timeSpentWaiting = (Timer.getFPGATimestamp() + 0.02 - waitStart)
            // + (CargoFetcher.getInstance().getNextTarget().timeSeconds
            // - currentTarget.timeSeconds);
            // } else if (currentTarget.type() == "Wait") {
            // Swerve.getInstance().setPathFollowerSpeeds(Swerve.ZERO_CHASSIS_SPEED);
            // timeSpentWaiting = isWaiting ? Timer.getFPGATimestamp() + 0.02 - waitStart :
            // timeSpentWaiting;

            // Path.Wait wait = (Path.Wait) currentTarget;

            // if (wait.getWaitType() == Path.Wait.WaitType.TIME) {
            // waitFinished = timeSpentWaiting >= wait.getWaitTime();
            // } else if (wait.getWaitType() == Path.Wait.WaitType.STATE) {
            // waitFinished = wait.isSatisfied();
            // }
            // }

            // if (waitFinished) {
            // isWaiting = false;
            // startTime = startTime + timeSpentWaiting;
            // timeSpentWaiting = -0.04;
            // waitFinished = true;
            // }
        } else {
            ChassisSpeeds speeds = Swerve.getInstance().getPathFollower()
                    .toPose(new Pose2d(currentTarget.poseMeters.getX(), currentTarget.poseMeters.getY(),
                            currentTarget.poseMeters.getRotation()));

            if (distanceFromEnd() < SwervePathFollower.FOLLOWER_END_DISTANCE_TOLERANCE) {
                speeds.vxMetersPerSecond = 0;
                speeds.vyMetersPerSecond = 0;
            }

            if (Swerve.getInstance().getControlState() == SwerveControlState.PATH_FOLLOWING)
                Swerve.getInstance().setPathFollowerSpeeds(speeds);
            // path.checkMarkers(Swerve.getInstance().getPoseMeters());

            if (currentIteration < totalIterations - 1)
                currentIteration++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED " + Units.metersToInches(distanceFromEnd()) + " INCHES FROM END");
        Swerve.getInstance().setPathFollowerSpeeds(Swerve.ZERO_CHASSIS_SPEED);
        Swerve.getInstance().stopPlottingRobotPath();
        Swerve.getInstance().stop();
    }

    private double distanceFromEnd() {
        return Swerve.getInstance().getPoseMeters().getTranslation()
                .getDistance(path.getStates().get(path.getStates().size() - 1).poseMeters
                        .getTranslation());
    }

    private double rotationFromEnd() {
        return Math.abs(Swerve.getInstance().getPoseMeters().getRotation()
                .minus(path.getStates().get(path.getStates().size() - 1).poseMeters.getRotation()).getDegrees());
    }

    public void setup() {
        Swerve.getInstance().resetOdometry(path.getStates().get(0).poseMeters);
    }

    @Override
    public boolean isFinished() {
        if (this.ignoreEndAngle)
            return Timer.getFPGATimestamp() - startTime >= endTime
                    && distanceFromEnd() < SwervePathFollower.FOLLOWER_END_DISTANCE_TOLERANCE;
        else
            return Timer.getFPGATimestamp() - startTime >= endTime
                    && distanceFromEnd() < SwervePathFollower.FOLLOWER_END_DISTANCE_TOLERANCE && rotationFromEnd() < 8;
    }
}