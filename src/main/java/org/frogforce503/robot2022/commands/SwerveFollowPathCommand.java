package org.frogforce503.robot2022.commands;

import java.util.function.Supplier;

import org.frogforce503.lib.follower.MarkerConfig;
import org.frogforce503.lib.follower.Path;
import org.frogforce503.lib.follower.SwervePathFollower;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveFollowPathCommand extends CommandBase {
    Path path;
    double endTime;
    boolean doesntWork = true;
    // Timer timer;
    // Timer waitTimer;
    int totalIterations;
    int currentIteration = 1;
    double startTime = 0;
    double waitStart = 0;
    double timeSpentWaiting = 0;
    boolean isWaiting = false;

    Pose2d startingState = new Pose2d();

    // double waitStartTime = 0;
    // int waitIterations = 0;
    // double waitBeginTime = 0;
    // boolean begunWaiting = false;
    // boolean waitCondition = false;

    // TODO: Make a way for the user of SwerveFollowPathCommand to call the
    // path.configureMarker function
    public SwerveFollowPathCommand(String pathFile) {
        path = Path.fromCSV(pathFile);
        SmartDashboard.putNumber(pathFile + " Path Length", path.getStates().size());
        endTime = path.getTotalTimeSeconds();
        totalIterations = path.getStates().size();
        // timer = new Timer();
        // waitTimer = new Timer();
    }

    public Pose2d end() {
        return path.getStates().get(path.getStates().size() - 1).poseMeters;
    }

    public void saveStartingState() {
        startingState = Swerve.getInstance().getPoseMeters();
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

    public SwerveFollowPathCommand setWaitCondition(Supplier<Boolean> condition) {
        this.path.setWaitCondition(condition);
        return this;
    }

    @SafeVarargs
    public final SwerveFollowPathCommand setWaitConditions(Supplier<Boolean>... condition) {
        SwerveFollowPathCommand cmd = this;

        for (Supplier<Boolean> c : condition) {
            cmd = cmd.setWaitCondition(c);
        }

        return cmd;
    }

    public SwerveFollowPathCommand configureMarkers(MarkerConfig config) {
        this.path.configureMarkers(config);
        return this;
    }

    // example for using wait condition with states.
    // public SwerveFollowPathCommand setWaitCondition(GameSpecState state) {
    // this.path.setWaitCondition(() -> GameSpec.getInstance().isAt(state));
    // }

    @Override
    public void initialize() {
        Swerve.getInstance().resetOdometry(path.getStates().get(0).poseMeters);
        Swerve.getInstance().drawPath(path);
        Swerve.getInstance().drawMarkers(path.getMarkers());
        Swerve.getInstance().startPlottingRobotPath();
        startTime = Timer.getFPGATimestamp();
        DriverFeedback.getInstance().setLEDState(LEDStates.BLUE);
        // startTime = timer.get();
        // timer.start();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp() - startTime - timeSpentWaiting;
        Path.State currentTarget = path.sample(currentTime);

        SmartDashboard.putNumber("X Setpoint", currentTarget.poseMeters.getX());
        SmartDashboard.putNumber("Y Setpoint", currentTarget.poseMeters.getY());

        path.checkMarkers(Swerve.getInstance().getPoseMeters());

        boolean waitFinished = false;

        if (currentTarget.type() == "Wait") {
            System.out.println("Third If");
            Swerve.getInstance().setPathFollowerSpeeds(Swerve.ZERO_CHASSIS_SPEED);
            timeSpentWaiting = isWaiting ? Timer.getFPGATimestamp() + 0.02 - waitStart : timeSpentWaiting;

            Path.Wait wait = (Path.Wait) currentTarget;

            if (wait.getWaitType() == Path.Wait.WaitType.TIME) {
                System.out.println("Fourth If");
                waitFinished = timeSpentWaiting >= wait.getWaitTime();
            } else if (wait.getWaitType() == Path.Wait.WaitType.STATE) {
                System.out.println("Fifth If");
                waitFinished = wait.isSatisfied();
            }
        }

        if (waitFinished) {
            System.out.println("Sixth If");
            isWaiting = false;
            startTime = startTime + timeSpentWaiting;
            timeSpentWaiting = -0.04;
            waitFinished = true;
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

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= endTime
                && distanceFromEnd() < SwervePathFollower.FOLLOWER_END_DISTANCE_TOLERANCE;
    }
}