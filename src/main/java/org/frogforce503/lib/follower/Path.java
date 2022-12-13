package org.frogforce503.lib.follower;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Represents a time-parameterized trajectory. The trajectory contains of
 * various States that
 * represent the pose, curvature, time elapsed, velocity, and acceleration at
 * that point.
 */
public class Path {
    private final double m_totalTimeSeconds;
    private final List<State> m_states;
    private final List<Marker> m_markers;
    private final List<Integer> m_waitIndices;

    private int lastSetWait = 0;

    /** Constructs an empty trajectory. */
    public Path() {
        m_states = new ArrayList<>();
        m_markers = new ArrayList<>();
        m_waitIndices = new ArrayList<>();
        m_totalTimeSeconds = 0.0;
    }

    /**
     * Constructs a trajectory from a vector of states and markers.
     *
     * @param states      A vector of states.
     * @param markers     A vector of markers.
     * @param waitIndices A vector of indices of states that are waits that require
     *                    a programmatic condtion(game spec state) (time-based waits
     *                    are not in this list)
     */
    public Path(final List<State> states, final List<Marker> markers, final List<Integer> waitIndices) {
        m_states = states;
        m_markers = markers;
        m_waitIndices = waitIndices;
        m_totalTimeSeconds = m_states.get(m_states.size() - 1).timeSeconds;
    }

    public Path(final List<State> states, final List<Marker> markers) {
        m_states = states;
        m_markers = markers;
        m_waitIndices = new ArrayList<>();
        m_totalTimeSeconds = m_states.get(m_states.size() - 1).timeSeconds;
    }

    public Path(final List<State> states) {
        m_states = states;
        m_markers = new ArrayList<>();
        m_waitIndices = new ArrayList<>();
        m_totalTimeSeconds = m_states.get(m_states.size() - 1).timeSeconds;
    }

    // @formatter:off
    public static Path fromCSV(String filename) {
        List<State> states = new ArrayList<>();
        List<Marker> markers = new ArrayList<>();
        List<Integer> waitIndices = new ArrayList<>();

        double timestamp = 0;
        int index = 0;

        try {
            List<String[]> parsedLines = Files
                    .lines(Paths.get(Filesystem.getDeployDirectory() + "/paths/" + filename
                            + (filename.endsWith(".csv") ? "" : ".csv")))
                    .map(line -> line.split(","))
                    .collect(Collectors.toList());

            for (String[] line : parsedLines) {
                try {
                    Double.parseDouble(line[0]);
                } catch (NumberFormatException e) {
                    // markers components are defined with + symbols, waits are used with - symbols
                    if (line[0].contains("+"))
                        markers = getMarkers(line);
                    else if (line[0].contains("-")) {
                        Wait w = markWait(line[0], timestamp, index);
                        
                        // if the wait is the type to require a condition, add it to the waitIndices
                        if (w.getWaitType() == Wait.WaitType.STATE)
                            waitIndices.add(Integer.valueOf(index));

                        states.add(w);
                    }
                    continue;
                }

                Pose2d poseMeters = new Pose2d(
                        Double.parseDouble(line[0]),
                        Units.feetToMeters((26 + 7 / 12)) - Double.parseDouble(line[1]), // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
                        Rotation2d.fromDegrees(Double.parseDouble(line[3]))
                                .unaryMinus()
                                // .rotateBy(new Rotation2d(Math.PI))
                );

                double velocity = Double.parseDouble(line[2]);

                State state = new State();

                state.poseMeters = poseMeters;
                state.velocityMetersPerSecond = velocity * 3;
                state.timeSeconds = timestamp;

                states.add(state);

                timestamp += 0.02;
                index++;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        for (Marker marker : markers)
            System.out.println(marker.toString());
        
        for (int waitIndex : waitIndices)
            System.out.println("Wait index at " + waitIndex);

        return new Path(states, markers, waitIndices);
    }

    // set the next Wait's condition to the given condition
    public void setWaitCondition(Supplier<Boolean> condition) {
        if (lastSetWait < m_waitIndices.size()) {
            int idx = m_waitIndices.get(lastSetWait);
            
            if (m_states.get(idx) instanceof Wait) {
                Wait wait = (Wait) m_states.get(idx);
                wait.setWaitCondition(condition);
                m_states.set(idx, wait);
                
                lastSetWait++;
            }
        }
    }

    // Method used by SwerveFollowPathCommand to configure the markers in this path from its inputs
    public void configureMarkers(MarkerConfig markerConfig) {
        markerConfig._configureMarkers(m_markers);
        markerConfig._registerSearchZones(this);
    }
    
    // internal method to mark a certain state as a wait and parse its type and (optional) time value
    private static Wait markWait(String line, double timeStamp, int idx) {
        String[] split = line.split("-");
        Wait.WaitType type = split[1].equals("Time") ? Wait.WaitType.TIME : Wait.WaitType.STATE;
        switch (type) {
            case TIME: {
                double time = Double.parseDouble(split[2]);
                return new Wait(timeStamp, time);
            }
            default: {
                Wait w = new Wait(timeStamp);
                w.setListIndex(idx);
                return w;
            }
        }
    }

    // get a list of markers generated from the CSV header that describes them
    private static List<Marker> getMarkers(String[] csvHeaderComponents) {
        ArrayList<Marker> markers = new ArrayList<Marker>();

        for (String unparsedMarker : csvHeaderComponents) {
            String[] markerComponents = unparsedMarker.split("\\+");

            System.out.println(markerComponents[0]);

            String name = markerComponents[0];
            Pose2d poseMeters = new Pose2d(Double.parseDouble(markerComponents[1]),
                    Double.parseDouble(markerComponents[2]), // 0.6 is value determined to fix weird
                                                                   // error in path planner output for the markers
                    Rotation2d.fromDegrees(0));
            double radius = Double.parseDouble(markerComponents[3]);

            Marker marker = new Marker(name, poseMeters, radius);
            markers.add(marker);
        }

        return markers;
    }

    // loop through each marker and check if the robot is interacting with it
    public void checkMarkers(Pose2d robotPose) {
        for (Marker marker : m_markers) {
            marker.periodic(robotPose);
        }
    }

    public void applySearchZoneWaits(List<Marker> searchZones) {
        ArrayList<State> markerEntrancePoints = new ArrayList<>();
        for (Marker marker : searchZones) {
       
        }

        ArrayList<State> newStates = new ArrayList<>();
        double offset = 0;
        for (State state : this.getStates()) {
            if (offset != 0) {
                state.timeSeconds += offset;
            }
            newStates.add(state);
            for (State markerEntrance : markerEntrancePoints) {
                if (markerEntrance.equals(state)) {
                    Wait wait = new Wait(state.timeSeconds + 0.02);
                    newStates.add(wait);
                    offset += 0.02;
                    break;
                }
            }
        }
    }

    /**
     * Linearly interpolates between two values.
     *
     * @param startValue The start value.
     * @param endValue   The end value.
     * @param t          The fraction for interpolation.
     * @return The interpolated value.
     */
    @SuppressWarnings("ParameterName")
    private static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    /**
     * Linearly interpolates between two poses.
     *
     * @param startValue The start pose.
     * @param endValue   The end pose.
     * @param t          The fraction for interpolation.
     * @return The interpolated pose.
     */
    @SuppressWarnings("ParameterName")
    private static Pose2d lerp(Pose2d startValue, Pose2d endValue, double t) {
        return startValue.plus((endValue.minus(startValue)).times(t));
    }

    /**
     * Returns the initial pose of the trajectory.
     *
     * @return The initial pose of the trajectory.
     */
    public Pose2d getInitialPose() {
        return sample(0).poseMeters;
    }

    /**
     * Returns the overall duration of the trajectory.
     *
     * @return The duration of the trajectory.
     */
    public double getTotalTimeSeconds() {
        return m_totalTimeSeconds;
    }

    /**
     * Return the states of the trajectory.
     *
     * @return The states of the trajectory.
     */
    public List<State> getStates() {
        return m_states;
    }

    public List<Marker> getMarkers() {
        return m_markers;
    }

    /**
     * Sample the trajectory at a point in time.
     *
     * @param timeSeconds The point in time since the beginning of the trajectory to
     *                    sample.
     * @return The state at that point in time.
     */
    public State sample(double timeSeconds) {
        if (timeSeconds <= m_states.get(0).timeSeconds) {
            return m_states.get(0);
        }
        if (timeSeconds >= m_totalTimeSeconds) {
            return m_states.get(m_states.size() - 1);
        }

        // To get the element that we want, we will use a binary search algorithm
        // instead of iterating over a for-loop. A binary search is O(std::log(n))
        // whereas searching using a loop is O(n).

        // This starts at 1 because we use the previous state later on for
        // interpolation.
        int low = 1;
        int high = m_states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (m_states.get(mid).timeSeconds < timeSeconds) {
                // This index and everything under it are less than the requested
                // timestamp. Therefore, we can discard them.
                low = mid + 1;
            } else {
                // t is at least as large as the element at this index. This means that
                // anything after it cannot be what we are looking for.
                high = mid;
            }
        }

        // High and Low should be the same.

        // The sample's timestamp is now greater than or equal to the requested
        // timestamp. If it is greater, we need to interpolate between the
        // previous state and the current state to get the exact state that we
        // want.
        final State sample = m_states.get(low);
        final State prevSample = m_states.get(low - 1);

        // if the sample that we are looking at is a Wait, we don't want to mess with interpolation
        if (sample.type() == "Wait") {
            return sample;
        } else if (prevSample.type() == "Wait") {
            return prevSample;
        }

        // If the difference in states is negligible, then we are spot on!
        if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-9) {
            return sample;
        }

        // Interpolate between the two states for the state that we want.
        return prevSample.interpolate(
                sample,
                (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
    }

    /**
     * Transforms all poses in the trajectory by the given transform. This is useful
     * for converting a
     * robot-relative trajectory into a field-relative trajectory. This works with
     * respect to the
     * first pose in the trajectory.
     *
     * @param transform The transform to transform the trajectory by.
     * @return The transformed trajectory.
     */
    @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
    public Path transformBy(Transform2d transform) {
        var firstState = m_states.get(0);
        var firstPose = firstState.poseMeters;

        // Calculate the transformed first pose.
        var newFirstPose = firstPose.plus(transform);
        List<State> newStates = new ArrayList<>();

        newStates.add(
                new State(
                        firstState.timeSeconds,
                        firstState.velocityMetersPerSecond,
                        firstState.accelerationMetersPerSecondSq,
                        newFirstPose,
                        firstState.curvatureRadPerMeter,
                        firstState.holonomicHeading));

        for (int i = 1; i < m_states.size(); i++) {
            var state = m_states.get(i);
            
            if (state.type() == "Wait")
                continue;
            // We are transforming relative to the coordinate frame of the new initial pose.
            newStates.add(
                    new State(
                            state.timeSeconds,
                            state.velocityMetersPerSecond,
                            state.accelerationMetersPerSecondSq,
                            newFirstPose.plus(state.poseMeters.minus(firstPose)),
                            state.curvatureRadPerMeter,
                            state.holonomicHeading));
        }

        return new Path(newStates);
    }

    /**
     * Transforms all poses in the trajectory so that they are relative to the given
     * pose. This is
     * useful for converting a field-relative trajectory into a robot-relative
     * trajectory.
     *
     * @param pose The pose that is the origin of the coordinate frame that the
     *             current trajectory
     *             will be transformed into.
     * @return The transformed trajectory.
     */
    public Path relativeTo(Pose2d pose) {
        return new Path(
                m_states.stream()
                        .map(
                                state -> new State(
                                        state.timeSeconds,
                                        state.velocityMetersPerSecond,
                                        state.accelerationMetersPerSecondSq,
                                        state.poseMeters.relativeTo(pose),
                                        state.curvatureRadPerMeter,
                                        state.holonomicHeading))
                        .collect(Collectors.toList()));
    }

    /**
     * Represents a time-parameterized trajectory. The trajectory contains of
     * various States that
     * represent the pose, curvature, time elapsed, velocity, and acceleration at
     * that point.
     */
    @SuppressWarnings("MemberName")
    public static class State {
        // The time elapsed since the beginning of the trajectory.
        @JsonProperty("time")
        public double timeSeconds;

        // The speed at that point of the trajectory. If the state is a wait state, this will be the time to wait
        @JsonProperty("velocity")
        public double velocityMetersPerSecond;

        // The acceleration at that point of the trajectory.
        @JsonProperty("acceleration")
        public double accelerationMetersPerSecondSq;

        // The pose at that point of the trajectory.
        @JsonProperty("pose")
        public Pose2d poseMeters;

        // The curvature at that point of the trajectory.
        @JsonProperty("curvature")
        public double curvatureRadPerMeter;

        // The curvature at that point of the trajectory.
        @JsonProperty("heading")
        public double holonomicHeading;

        public State() {
            poseMeters = new Pose2d();
        }

        /**
         * Constructs a State with the specified parameters.
         *
         * @param timeSeconds                   The time elapsed since the beginning of
         *                                      the trajectory.
         * @param velocityMetersPerSecond       The speed at that point of the
         *                                      trajectory.
         * @param accelerationMetersPerSecondSq The acceleration at that point of the
         *                                      trajectory.
         * @param poseMeters                    The pose at that point of the
         *                                      trajectory.
         * @param curvatureRadPerMeter          The curvature at that point of the
         *                                      trajectory.
         * @param holonomicHeading              The actual holonomic robot heading at
         *                                      that point of the trajectory.
         */
        public State(
                double timeSeconds,
                double velocityMetersPerSecond,
                double accelerationMetersPerSecondSq,
                Pose2d poseMeters,
                double curvatureRadPerMeter,
                double holonomicHeading) {
            this.timeSeconds = timeSeconds;
            this.velocityMetersPerSecond = velocityMetersPerSecond;
            this.accelerationMetersPerSecondSq = accelerationMetersPerSecondSq;
            this.poseMeters = poseMeters;
            this.curvatureRadPerMeter = curvatureRadPerMeter;
            this.holonomicHeading = holonomicHeading;
        }

        /**
         * Interpolates between two States.
         *
         * @param endValue The end value for the interpolation.
         * @param i        The interpolant (fraction).
         * @return The interpolated state.
         */
        @SuppressWarnings("ParameterName")
        State interpolate(State endValue, double i) {
            // Find the new t value.
            final double newT = lerp(timeSeconds, endValue.timeSeconds, i);

            // Find the delta time between the current state and the interpolated state.
            final double deltaT = newT - timeSeconds;

            // If delta time is negative, flip the order of interpolation.
            if (deltaT < 0) {
                return endValue.interpolate(this, 1 - i);
            }

            // Check whether the robot is reversing at this stage.
            final boolean reversing = velocityMetersPerSecond < 0
                    || Math.abs(velocityMetersPerSecond) < 1E-9 && accelerationMetersPerSecondSq < 0;

            // Calculate the new velocity
            // v_f = v_0 + at
            final double newV = velocityMetersPerSecond + (accelerationMetersPerSecondSq * deltaT);

            // Calculate the change in position.
            // delta_s = v_0 t + 0.5 at^2
            final double newS = (velocityMetersPerSecond * deltaT
                    + 0.5 * accelerationMetersPerSecondSq * Math.pow(deltaT, 2))
                    * (reversing ? -1.0 : 1.0);

            // Return the new state. To find the new position for the new state, we need
            // to interpolate between the two endpoint poses. The fraction for
            // interpolation is the change in position (delta s) divided by the total
            // distance between the two endpoints.
            final double interpolationFrac = newS
                    / endValue.poseMeters.getTranslation().getDistance(poseMeters.getTranslation());

            return new State(
                    newT,
                    newV,
                    accelerationMetersPerSecondSq,
                    lerp(poseMeters, endValue.poseMeters, interpolationFrac),
                    lerp(curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac),
                    lerp(holonomicHeading, endValue.holonomicHeading, interpolationFrac));
        }

        @Override
        public String toString() {
            return String.format(
                    "State(Sec: %.2f, Vel m/s: %.2f, Accel m/s/s: %.2f, Pose: %s, Curvature: %.2f)",
                    timeSeconds,
                    velocityMetersPerSecond,
                    accelerationMetersPerSecondSq,
                    poseMeters,
                    curvatureRadPerMeter);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof State)) {
                return false;
            }
            State state = (State) obj;
            return Double.compare(state.timeSeconds, timeSeconds) == 0
                    && Double.compare(state.velocityMetersPerSecond, velocityMetersPerSecond) == 0
                    && Double.compare(state.accelerationMetersPerSecondSq, accelerationMetersPerSecondSq) == 0
                    && Double.compare(state.curvatureRadPerMeter, curvatureRadPerMeter) == 0
                    && Objects.equals(poseMeters, state.poseMeters)
                    && state.type().equals(type());
        }

        @Override
        public int hashCode() {
            return Objects.hash(
                    timeSeconds,
                    velocityMetersPerSecond,
                    accelerationMetersPerSecondSq,
                    poseMeters,
                    curvatureRadPerMeter);
        }

        public String type() {
            return "PathState";
        }
    }

    /**
     * Represents any type of Wait in a path, either Time-based or State-based.
     * 
     * Time based means that the robot will wait for a certain number of seconds at the point.
     * State based means that the robot will wait until a given waitCondition function returns true; this should
     * be used in conjunction with the game spec state engine to sync robot movements with the robot's overall state
     * 
     * This class is a child of the State class meaning that it can be in the list of States and be accessed as such.
     */
    public static class Wait extends State {
        public static enum WaitType {
            TIME,
            STATE
        }

        private final WaitType waitType;
        private Supplier<Boolean> waitCondition = null;
        private int listIndex = 0;

        public Wait(double timestamp, double waitTime) {
            super(timestamp, 0, 0, new Pose2d(503503, waitTime, Rotation2d.fromDegrees(0)), 0, 0);
            waitType = WaitType.TIME;
        }

        public int getListIndex() {
            return listIndex;
        }

        public void setListIndex(int listIndex) {
            this.listIndex = listIndex;
        }

        public Wait(double timestamp) {
            super(timestamp, 0, 0, new Pose2d(503503, 503503, Rotation2d.fromDegrees(0)), 0, 0);
            waitType = WaitType.STATE;
        }

        public void setWaitCondition(Supplier<Boolean> waitCondition) {
            System.out.println("Wait condition has been set");
            this.waitCondition = waitCondition;
        }

        public boolean isSatisfied() {
            System.out.println("IS THE WAIT CONDITION NULLL? " + waitCondition == null);
            return waitCondition != null ? waitCondition.get() : true;
        }

        public double getWaitTime() {
            return this.poseMeters.getY();
        }

        public WaitType getWaitType() {
            return waitType;
        }

        @Override
        public String toString() {
            return String.format("Wait(%s)", waitType);
        }

        @Override
        public String type() {
            return "Wait";
        }
    }

    /**
     * Represents a space-based marker on a Path.
     * This will trigger a callback when the robot center is within the range of the
     * marker.
     */
    public static class Marker {
        public String name;
        public Pose2d poseMeters;
        public double radius;

        private Runnable onEnter;
        private Runnable insidePeriodic;
        private Runnable onExit;

        public boolean wasInRange;
        public boolean inRange;

        public Supplier<Boolean> stateSZSupplier; // ONLY used by Search Zones that are not actually searching (yes this needs to be cleaned up)
        public boolean isUnconventionalSZ;

        private boolean disabled = false;

        public Marker(String name, Pose2d poseMeters, double radius) {
            this.name = name;
            this.poseMeters = poseMeters;
            this.radius = radius;
        }

        public void setFunctions(Runnable onEnter, Runnable insidePeriodic, Runnable onExit) {
            this.onEnter = onEnter;
            this.insidePeriodic = insidePeriodic;
            this.onExit = onExit;
        }

        public boolean isInRange(Pose2d robotPose) {
            return robotPose.getTranslation().getDistance(poseMeters.getTranslation()) <= radius;
        }

        public void periodic(Pose2d robotPose) {
            if (disabled)
                return;

            
            inRange = isInRange(robotPose);

            if (inRange && !wasInRange) {
                if (onEnter != null) {
                    onEnter.run();
                }
            } else if (!inRange && wasInRange) {
                if (onExit != null) {
                    onExit.run();
                    disable();
                }
            } else if (inRange && wasInRange) {
                if (insidePeriodic != null) {
                    insidePeriodic.run();
                }
            }

            wasInRange = inRange;
        }

        public void disable() {
            this.disabled = true;
        }

        public String markerType() {
            return "Marker";
        }

        public boolean isUnconventionalSearchZone() {
            return this.isUnconventionalSZ;
        }

        @Override
        public String toString() {
            return String.format("Marker(%s, %.2f, %.2f, %.2f)", name, poseMeters.getX(), poseMeters.getY(), radius);
        }

        @Override
        public int hashCode(){
            return this.toString().hashCode();
        }

        @Override
        public boolean equals(Object o) {
            return this.toString().equals(o.toString());
        }
    }

    @Override
    public String toString() {
        String stateList = m_states.stream().map(State::toString).collect(Collectors.joining(", \n"));
        return String.format("Trajectory - Seconds: %.2f, States:\n%s", m_totalTimeSeconds, stateList);
    }

    @Override
    public int hashCode() {
        return m_states.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Path && m_states.equals(((Path) obj).getStates());
    }
}