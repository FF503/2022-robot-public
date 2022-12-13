package org.frogforce503.robot2022.subsystems.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;

import org.frogforce503.lib.follower.Path;
import org.frogforce503.lib.follower.Path.Marker;
import org.frogforce503.lib.follower.SwervePathFollower;
import org.frogforce503.robot2022.OI;
import org.frogforce503.robot2022.Robot;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.Subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem {
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SwerveModule[] modules;
    private Pigeon2 imu;

    // math
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    // controllers
    private SwervePathFollower pathFollower;
    private ProfiledPIDController stabilizationController;

    private double stabilizationP = 0; // FIXME: tune this pid
    private double stabilizationI = 0;
    private double stabilizationD = 0;

    static Swerve instance;

    // simulation stuff (field is technically used in both sim and real)
    private Field2d field;
    private double _simHeading;
    private boolean _isPlottingRobotPose = false;

    // constants
    private final double SLOW_TRANSLATION_METERS_PER_SECOND = 1.5;
    private final double FAST_TRANSLATION_METERS_PER_SECOND = 4;

    private final double SLOW_ROTATION_RADIANS_PER_SECOND = Math.PI / 2; // 90 deg per second
    private final double FAST_ROTATION_RADIANS_PER_SECOND = 2.5 * (Math.PI / 2); // 270 deg per second

    public static final double JOYSTICK_DRIVE_TOLERANCE = 0.1;
    public static final double JOYSTICK_TURN_TOLERANCE = 0.1;

    public static final ChassisSpeeds ZERO_CHASSIS_SPEED = new ChassisSpeeds();
    private final Rotation2d DEFAULT_STABALIZATION_HEADING = Rotation2d.fromDegrees(503.503503);

    // current state of drivetrain
    private Pose2d currentVelocity = new Pose2d();
    private double lastTime;
    private Rotation2d lastTheta;
    private boolean brakeModeEnabled = false;

    // control of the robot
    private SwerveControlState swerveControlState = SwerveControlState.STOPPED;
    private ChassisSpeeds pathFollowerSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds teleopChassisSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds fetchingChassisSpeeds = ZERO_CHASSIS_SPEED;

    // teleop values
    private Rotation2d stabilizationHeading = DEFAULT_STABALIZATION_HEADING;
    private boolean slowmodeEnabled = false;
    private boolean isRobotCentric = false;
    private boolean isSnapping = false;
    private boolean drivingAboveTolerance = false;

    private boolean isCargoFetching = false;
    private double fetchingXConst;
    private double fetchingYConst;
    private double fetchingThetaConst;

    private ShuffleboardLayout swerveTableList;
    private NetworkTable swerveTable;

    SlewRateLimiter xFilter = new SlewRateLimiter(1, 0);
    SlewRateLimiter yFilter = new SlewRateLimiter(1, 0);

    /**
     * Creates a new Drive.
     */
    public Swerve() {
        this.frontLeftModule = new SwerveModule(Robot.bot.frontLeftName, ModuleLocation.FrontLeft);
        this.frontRightModule = new SwerveModule(Robot.bot.frontRightName, ModuleLocation.FrontRight);
        this.backLeftModule = new SwerveModule(Robot.bot.backLeftName, ModuleLocation.BackLeft);
        this.backRightModule = new SwerveModule(Robot.bot.backRightName, ModuleLocation.BackRight);

        this.modules = new SwerveModule[] {
                this.frontLeftModule,
                this.frontRightModule,
                this.backLeftModule,
                this.backRightModule
        };

        field = new Field2d();
        field.getObject("ball").setPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0)));

        SmartDashboard.putData("Field", field);

        imu = new Pigeon2(Robot.bot.pigeonID);
        imu.configFactoryDefault();
        setAngle(0);

        Translation2d m_frontLeftLocation = new Translation2d(Robot.bot.kWheelbaseWidth, Robot.bot.kWheelbaseWidth);
        Translation2d m_frontRightLocation = new Translation2d(Robot.bot.kWheelbaseWidth, -Robot.bot.kWheelbaseWidth);
        Translation2d m_backLeftLocation = new Translation2d(-Robot.bot.kWheelbaseWidth, Robot.bot.kWheelbaseWidth);
        Translation2d m_backRightLocation = new Translation2d(-Robot.bot.kWheelbaseWidth, -Robot.bot.kWheelbaseWidth);

        kinematics = new SwerveDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        odometry = new SwerveDriveOdometry(kinematics, getAngleRotation2d());

        stabilizationController = new ProfiledPIDController(5.0, 0, 0.0,
                // stabilizationController = new ProfiledPIDController(0.0, 0, 0.0,
                new TrapezoidProfile.Constraints(2.5 * (Math.PI / 2), 25 * (Math.PI / 2)));

        pathFollower = new SwervePathFollower(
                new PIDController(4.5, 0, 0),
                new PIDController(8, 0, 0.0),
                new ProfiledPIDController(1, 0, 0,
                        new TrapezoidProfile.Constraints(6.28 * 2.835, 3.14 * 2)));

        this.stabilizationController.setTolerance(Math.toRadians(4));
        this.stabilizationController.enableContinuousInput(-Math.PI, Math.PI);
        this.lastTime = Timer.getFPGATimestamp();
        this.lastTheta = getAngleRotation2d();

        for (SwerveModule module : modules) {
            SmartDashboard.putString(module.locationName + " ID", module.moduleName);
        }

        // Shuffleboard.getTab("GameSpec").addString("Search Times", () ->
        // this.searchTimes + "");
        // Shuffleboard.getTab("GameSpec").addString("Disable Times", () ->
        // this.disableTimes + "");
        // Shuffleboard.getTab("GameSpec").addBoolean("CargoFetcher Sees Ball",
        // CargoFetcher.getInstance()::isCurrentlySearching);

        swerveTableList = Shuffleboard.getTab("GameSpec").getLayout("Swerve", BuiltInLayouts.kList);
        swerveTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Swerve");

        swerveTableList.withPosition(10, 0).withSize(2, 5);
    }

    public static Swerve getInstance() {
        return instance == null ? instance = new Swerve() : instance;
    }

    public void initTelemetry() {
        swerveTableList.withPosition(0, 0).withSize(2, 5);

        swerveTableList
                .addNumber("Stabilization P", () -> swerveTable.getEntry("Stabilization P").getDouble(stabilizationP))
                .withSize(1, 1);
        swerveTableList
                .addNumber("Stabilization I", () -> swerveTable.getEntry("Stabilization I").getDouble(stabilizationI))
                .withSize(1, 1);
        swerveTableList
                .addNumber("Stabilization D", () -> swerveTable.getEntry("Stabilization D").getDouble(stabilizationD))
                .withSize(1, 1);
    }

    public void updatePIDValues() {
        stabilizationController.setP(swerveTable.getEntry("Stabilization P").getDouble(0.0));
        stabilizationController.setI(swerveTable.getEntry("Stabilization I").getDouble(0.0));
        stabilizationController.setD(swerveTable.getEntry("Stabilization D").getDouble(0.0));
    }

    @Override
    public void writePeriodicOutputs() {
        updateOdometry();
        this.drawFields();
        this.control();

        if (RobotBase.isSimulation())
            this.simulationPeriodic();
    }

    /**
     * Main control function that is called in write periodic outputs. This should
     * be the ONLY place that Swerve.drive is ever called
     */
    private void control() {
        ChassisSpeeds speeds;
        switch (this.swerveControlState) {
            case TELEOP:
                this.driveWithStabilization();
                cleanseChassisSpeeds(teleopChassisSpeeds);
                speeds = this.teleopChassisSpeeds;
                break;
            case PATH_FOLLOWING:
                cleanseChassisSpeeds(this.pathFollowerSpeeds);
                speeds = this.pathFollowerSpeeds;
                break;
            case BALL_FETCHING:
                DriverFeedback.getInstance().setLEDState(LEDStates.RAINBOW);
                this.driveWithStabilization();
                cleanseChassisSpeeds(teleopChassisSpeeds);
                cleanseChassisSpeeds(fetchingChassisSpeeds);

                fetchingChassisSpeeds.vxMetersPerSecond += teleopChassisSpeeds.vxMetersPerSecond;
                fetchingChassisSpeeds.vyMetersPerSecond += teleopChassisSpeeds.vyMetersPerSecond;
                fetchingChassisSpeeds.omegaRadiansPerSecond += teleopChassisSpeeds.omegaRadiansPerSecond;

                speeds = fetchingChassisSpeeds;
                break;
            case SNAPPING:
                // continue to zero chassis speed
            default:
                // this is stopped mode (snapping also does this stuff)
                speeds = ZERO_CHASSIS_SPEED;
                checkForStateChange();
                break;
        }

        this.drive(speeds);
    }

    private void checkForStateChange() {
        if (RobotState.isTeleop() && OI.tryingToDrive()) {
            this.setControlState(SwerveControlState.TELEOP);
        }
        if (RobotState.isAutonomous()) { // TODO: Fix this to work with path planner stopping
            this.setControlState(SwerveControlState.PATH_FOLLOWING);
        }
    }

    //@formatter:on
    int searchTimes = 0;
    int disableTimes = 0;

    public void setControlState(SwerveControlState state) {
        if (state == SwerveControlState.BALL_FETCHING)
            searchTimes++;
        if (this.swerveControlState == SwerveControlState.BALL_FETCHING && state != SwerveControlState.BALL_FETCHING) {
            disableTimes++;
            // for (var el : Thread.currentThread().getStackTrace()) {
            // System.out.println(el.toString());
            // }
            // System.exit(0);
        }
        // System.out.println("SETTING CONTROL STATE TO: " + state.name());
        this.swerveControlState = state;
    }

    public SwerveControlState getControlState() {
        return this.swerveControlState;
    }

    /**
     * Disables cargo fetcher and switches control mode to teleop or path following
     * based on the match state.
     */
    public void disableFetcher() {
        if (this.swerveControlState == SwerveControlState.BALL_FETCHING) {
            // disableTimes++;
            this.fetchingChassisSpeeds = new ChassisSpeeds(0, 0, 0);
            if (RobotState.isTeleop())
                this.setControlState(SwerveControlState.TELEOP);
            else if (RobotState.isAutonomous())
                this.setControlState(SwerveControlState.PATH_FOLLOWING);
            else
                this.setControlState(SwerveControlState.STOPPED);
        }
    }

    /**
     * Draws the path and the robot on the dashboard.
     */
    private void drawFields() {

        List<Pose2d> ballList = new ArrayList<>();
        field.getObject("ball").setPoses(ballList);
        field.setRobotPose(getPoseMeters());
        if (_isPlottingRobotPose)
            this.plotRobotPathPoint();
    }

    public void toggleSlowMode() {
        this.slowmodeEnabled = !slowmodeEnabled;
    }

    public void toggleRobotCentric() {
        this.isRobotCentric = !isRobotCentric;
    }

    public void setCargoFetching(boolean fetching) {
        this.isCargoFetching = fetching;
    }

    /**
     * Snap the ROBOT to a heading (this will only work in Teleop as of now)
     * 
     * @param angle Angle in degres to snap to
     */
    public void snapToAngle(double angle) {
        stabilizationHeading = Rotation2d.fromDegrees(angle);
        stabilizationController.reset(stabilizationHeading.getRadians());
        isSnapping = true;
    }

    public void disableSnapping() {
        isSnapping = false;
    }

    /**
     * Main teleop control loop
     */
    private void driveWithStabilization() {

        // dont read user input in autonomous
        if (RobotState.isAutonomous()) {
            this.teleopChassisSpeeds = ZERO_CHASSIS_SPEED;
            return;
        }

        double rightStickX = -OI.getDriverRightXValue();
        double leftStickY = OI.getDriverLeftYValue();
        double leftStickX = -OI.getDriverLeftXValue();

        drivingAboveTolerance = (Math.hypot(leftStickY, leftStickX) > JOYSTICK_DRIVE_TOLERANCE);

        if (!drivingAboveTolerance) {
            leftStickX = 0;
            leftStickY = 0;
        }

        if (isSnapping) {
            setStabilizationP(2.0);
            setStabilizationI(0.0001);
        } else {
            // setStabilizationP(0.0);
            // setStabilizationI(0.0);
        }

        // these 3 lines out is testing. normally should be inside else of
        // rotationoutput less than joystick turn tolerance
        stabilizationHeading = getAngleRotation2d();
        // setStabilizationI(0);
        // setStabilizationP(0);
        stabilizationController.setGoal(stabilizationHeading.getRadians());
        stabilizationController.reset(stabilizationHeading.getRadians());

        double rotationOutput;
        if (!isCargoFetching) {
            rotationOutput = rightStickX;
            if (Math.abs(rotationOutput) < JOYSTICK_TURN_TOLERANCE) {
                rotationOutput = (drivingAboveTolerance || isSnapping)
                        ? stabilizationController.calculate(getAngleRotation2d().getRadians(),
                                stabilizationHeading.getRadians())
                        : 0;

                swerveTable.getEntry("Rotation output").setValue(rotationOutput);

                if (!drivingAboveTolerance && !isSnapping) {
                    stabilizationHeading = getAngleRotation2d();
                    stabilizationController.reset(stabilizationHeading.getRadians());
                }

                // if (isSnapping &&
                // Math.abs(getAngleRotation2d().minus(stabilizationHeading).getDegrees()) < 1)
                // {
                // isSnapping = false;
                // }
            } else {
                rotationOutput *= this.slowmodeEnabled ? SLOW_ROTATION_RADIANS_PER_SECOND
                        : FAST_ROTATION_RADIANS_PER_SECOND;
                isSnapping = false;
            }
        } else {
            rotationOutput = this.fetchingChassisSpeeds.omegaRadiansPerSecond;
        }

        double xVel = (-leftStickY
                * (this.slowmodeEnabled ? SLOW_TRANSLATION_METERS_PER_SECOND : FAST_TRANSLATION_METERS_PER_SECOND));
        double yVel = (leftStickX
                * (this.slowmodeEnabled ? SLOW_TRANSLATION_METERS_PER_SECOND : FAST_TRANSLATION_METERS_PER_SECOND));

        SmartDashboard.putNumber("xSpeed", xVel);
        SmartDashboard.putNumber("ySpeed", yVel);
        Translation2d corrections = new Translation2d(xVel, yVel);

        if (this.isRobotCentric) {
            teleopChassisSpeeds.vxMetersPerSecond = xVel;
            teleopChassisSpeeds.vyMetersPerSecond = yVel;
            teleopChassisSpeeds.omegaRadiansPerSecond = rotationOutput;
        } else {
            teleopChassisSpeeds = (ChassisSpeeds.fromFieldRelativeSpeeds(corrections.getX(),
                    corrections.getY(),
                    rotationOutput, getAngleRotation2d()));
        }

        double xVelAbs = Math.abs(teleopChassisSpeeds.vxMetersPerSecond);
        double yVelAbs = Math.abs(teleopChassisSpeeds.vyMetersPerSecond);
        double oVelAbs = Math.abs(teleopChassisSpeeds.omegaRadiansPerSecond);

        if (!isSnapping && swerveControlState != SwerveControlState.BALL_FETCHING && xVelAbs < 0.05 && yVelAbs < 0.05
                && oVelAbs < 0.05
                && getVelocity().getTranslation().getNorm() < 0.05
                && getVelocity().getRotation().getRadians() < 0.05) {
            snapModulesTo(ModuleSnapPositions.DEFENSE);
        }

    }

    /**
     * Generated ChassisSpeeds objects that are obviously too low will be ignored
     * 
     * @param speeds ChassisSpeeds object to be cleaned
     */
    private void cleanseChassisSpeeds(ChassisSpeeds speeds) {
        speeds.vxMetersPerSecond = Math.abs(speeds.vxMetersPerSecond) < 0.05 ? 0 : speeds.vxMetersPerSecond;
        speeds.vyMetersPerSecond = Math.abs(speeds.vyMetersPerSecond) < 0.05 ? 0 : speeds.vyMetersPerSecond;
        speeds.omegaRadiansPerSecond = Math.abs(speeds.omegaRadiansPerSecond) < Math.toRadians(1) ? 0
                : speeds.omegaRadiansPerSecond;
    }

    private void simulationPeriodic() {
        for (SwerveModule module : modules) {
            module.simulationPeriodic();
        }
    }

    public void resetStabilizationHeading() {
        if (stabilizationHeading.equals(DEFAULT_STABALIZATION_HEADING))
            stabilizationHeading = getAngleRotation2d();
    }

    public boolean isDrivingAboveTolerance() {
        return RobotState.isTeleop() && drivingAboveTolerance;
    }

    public void enableBrakeMode(boolean enable) {
        for (SwerveModule module : modules) {
            module.enableBrakeMode(enable);
        }
        brakeModeEnabled = enable;
    }

    public boolean getBrakeMode() {
        return brakeModeEnabled;
    }

    public void setAngle(double angle) {
        imu.setYaw(angle);
        stabilizationHeading = Rotation2d.fromDegrees(angle);

        if (stabilizationController != null)
            stabilizationController.reset(stabilizationHeading.getRadians());

        _simHeading = angle;
    }

    public void setStabilizationP(double P) {
        this.stabilizationController.setP(P);
    }

    public void setStabilizationI(double I) {
        this.stabilizationController.setI(I);
    }

    public double getSnappingGain() {
        return stabilizationController.calculate(getAngleRotation2d().getRadians(), stabilizationHeading.getRadians());
    }

    public HashMap<String, Double> getModuleRezeroValues() {
        HashMap<String, Double> rezeroValues = new HashMap<>();

        for (SwerveModule module : modules) {
            rezeroValues.put(module.moduleName, module.getRezeroValue());
        }

        return rezeroValues;
    }

    public double getPitch() {
        double angle = imu.getPitch();
        return angle; // used to be a negative, taken out to account for pigeon increase direction.
    }

    /**
     * Get the current angle of the robot in degrees
     * 
     * @return double (degrees)
     */
    public double getAngleDegrees() {
        double angle = (RobotBase.isReal() ? imu.getYaw() : _simHeading) % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle; // used to be a negative, taken out to account for pigeon increase direction.
    }

    /**
     * Get the current angle of the robot as a Rotation2d
     * 
     * @return Rotation2D object
     */
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(getAngleDegrees());
    }

    public void updateOdometry() {
        // for (SwerveModule module : this.modules)
        // module.readPeriodicInputs();

        double dt = Timer.getFPGATimestamp() - lastTime;
        Pose2d lastPose = getPoseMeters();
        odometry.update(getAngleRotation2d(), frontLeftModule.getSwerveModuleState(),
                frontRightModule.getSwerveModuleState(), backLeftModule.getSwerveModuleState(),
                backRightModule.getSwerveModuleState());

        Pose2d curPose = odometry.getPoseMeters();
        currentVelocity = new Pose2d((curPose.getX() - lastPose.getX()) / dt, (curPose.getY() - lastPose.getY()) / dt,
                getAngleRotation2d().minus(lastTheta).times(1 / dt));

        lastTime = Timer.getFPGATimestamp();
        lastTheta = getAngleRotation2d();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
        setAngle(pose.getRotation().getDegrees());
    }

    public void setModuleDrivePct(double pct) {
        for (SwerveModule module : modules) {
            module.setDriveMotor(ControlMode.PercentOutput, pct);
        }
    }

    public void setDriveModuleVelocity(double vel) {
        for (SwerveModule module : modules) {
            module.setDriveMotor(ControlMode.Velocity, vel);
        }
    }

    /**
     * Snap all the swerve modules to a specified preset
     */
    public void snapModulesTo(ModuleSnapPositions preset) {
        setControlState(SwerveControlState.SNAPPING);
        int idx = 0;
        for (SwerveModule module : modules) {
            module.setDriveMotor(ControlMode.PercentOutput, 0);
            module.setRotationPosition(Rotation2d.fromDegrees(preset.getSnapPositions()[idx]));
            idx++;
        }
    }

    /**
     * Sets the rotation of ALL modules to the specified angle.
     * It is preferred to use the snapModulesTo method instead.
     */
    public void setModuleRotation(double degrees) {
        setControlState(SwerveControlState.SNAPPING);
        for (SwerveModule module : modules) {
            module.setRotationPosition(Rotation2d.fromDegrees(degrees));
        }
    }

    /**
     * ONLY TO BE CALLED BY THE CONTROL FUNCTION
     * 
     * @param chassisSpeeds
     */
    private void drive(ChassisSpeeds chassisSpeeds) {
        if (swerveControlState == SwerveControlState.SNAPPING)
            return;

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.MAX_DRIVE_SPEED_METERS_SEC);
        frontLeftModule.setSwerveModuleState(moduleStates[0], swerveControlState.rememberLastPosition);
        frontRightModule.setSwerveModuleState(moduleStates[1], swerveControlState.rememberLastPosition);
        backLeftModule.setSwerveModuleState(moduleStates[2], swerveControlState.rememberLastPosition);
        backRightModule.setSwerveModuleState(moduleStates[3], swerveControlState.rememberLastPosition);
        _simHeading += chassisSpeeds.omegaRadiansPerSecond;
    }

    /**
     * Update the speeds that the path follower determines are necessary to follow
     * the path
     * Should be used in Auton mostly unless if we decided to add it into teleop or
     * tests as well.
     * 
     * @param chassisSpeeds The speeds to drive the robot at
     */
    public void setPathFollowerSpeeds(ChassisSpeeds chassisSpeeds) {
        this.pathFollowerSpeeds = chassisSpeeds;
        // System.out.println("Setting Path FOllower Speeds " +
        // CargoFetcher.getInstance().isCurrentlySearching());
        this.setControlState(SwerveControlState.PATH_FOLLOWING);
    }

    // private void drive(double xVelMeters, double yVelMeters, double
    // degreesPerSecond, boolean isFieldRelative) {
    // if (isFieldRelative) {
    // drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters,
    // Math.toRadians(degreesPerSecond),
    // getAngleRotation2d()));
    // } else {
    // drive(new ChassisSpeeds(xVelMeters, yVelMeters,
    // Math.toRadians(degreesPerSecond)));
    // }
    // }

    public void drawPath(Path path) {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (Path.State state : path.getStates()) {
            poses.add(state.poseMeters);
        }
        field.getObject("path").setPoses(poses);
    }

    public void drawMarkers(List<Marker> markers) {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (Marker marker : markers) {
            Pose2d upright = new Pose2d(marker.poseMeters.getX(), marker.poseMeters.getY() + marker.radius,
                    Rotation2d.fromDegrees(0));
            for (int i = 0; i < 360; i++) {
                Pose2d circlePoint = upright
                        .relativeTo(new Pose2d(marker.poseMeters.getTranslation(), Rotation2d.fromDegrees(i)));
                // .plus(new Transform2d(marker.poseMeters.getTranslation(),
                // Rotation2d.fromDegrees(0)));
                Pose2d circlePointField = new Pose2d(marker.poseMeters.getX() + circlePoint.getX(),
                        marker.poseMeters.getY()
                                + circlePoint.getY(),
                        circlePoint.getRotation());
                poses.add(circlePointField);
            }
            poses.add(marker.poseMeters);
        }
        field.getObject("markers").setPoses(poses);
    }

    public void startPlottingRobotPath() {
        this._isPlottingRobotPose = true;
    }

    public void stopPlottingRobotPath() {
        this._isPlottingRobotPose = false;
    }

    private void plotRobotPathPoint() {
        List<Pose2d> temp = field.getObject("robotpath").getPoses();
        temp.add(getPoseMeters());

        field.getObject("robotpath").setPoses(temp);
    }

    public void clearPath() {
        System.out.println("Disabled");
        field.getObject("path").setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, Rotation2d.fromDegrees(0)) }));
        field.getObject("robotpath")
                .setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, Rotation2d.fromDegrees(0)) }));
        field.getObject("markers")
                .setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, Rotation2d.fromDegrees(0)) }));
        this._isPlottingRobotPose = false;
    }

    // public ChassisSpeeds getTrajectoryFollowerOutput(Trajectory.State target,
    // double angle) {
    // return controller.calculate(getPoseMeters(), target,
    // Rotation2d.fromDegrees(angle));
    // }

    public Pose2d getPoseMeters() {
        return new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), getAngleRotation2d());
    }

    public Pose2d getVelocity() {
        return this.currentVelocity;
    }

    public Rotation2d getGyroRotationalVelocity() {
        double[] xyz_dps = new double[3];
        imu.getRawGyro(xyz_dps);
        return Rotation2d.fromDegrees(xyz_dps[2]);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Robot X: ", getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y: ", getPoseMeters().getY());
        SmartDashboard.putNumber("Robot Heading: ", getAngleDegrees());
        SmartDashboard.putString("SWERVE CONTROL STATE: ",
                this.swerveControlState.name());
        SmartDashboard.putBoolean("Is Snapping: ", isSnapping);

        // SmartDashboard.putNumber("Stabalization Target: ",
        // stabilizationHeading.getDegrees());
        // // SmartDashboard.putNumber("Driver Left Stick X: ",
        // OI.getDriverLeftXValue());
        // // SmartDashboard.putNumber("Driver Left Stick Y: ",
        // OI.getDriverLeftYValue());
        // // SmartDashboard.putNumber("Driver Right Stick X: ",
        // // OI.getDriverRightXValue());

        // SmartDashboard.putNumber("Measured Angular Velocity: ",
        // currentVelocity.getRotation().getDegrees());
        // SmartDashboard.putNumber("Measured X Velocity: ", currentVelocity.getX());
        // SmartDashboard.putNumber("Measured Y Velocity: ", currentVelocity.getX());

        // SmartDashboard.putBoolean("Robot Centric Enabled", isRobotCentric);
        // SmartDashboard.putBoolean("Slow Mode Enabled", slowmodeEnabled);

        // SmartDashboard.putNumber("Fetching chassis speeds X",
        // this.fetchingChassisSpeeds.vxMetersPerSecond);

        for (SwerveModule module : this.modules) {
            module.outputTelemetry();
        }
    }

    @Override
    public void stop() {
        for (SwerveModule module : this.modules) {
            module.stop();
        }

        this.stabilizationHeading = DEFAULT_STABALIZATION_HEADING;

        this.pathFollowerSpeeds = ZERO_CHASSIS_SPEED;
        this.teleopChassisSpeeds = ZERO_CHASSIS_SPEED;
        this.fetchingChassisSpeeds = ZERO_CHASSIS_SPEED;

        this.setControlState(SwerveControlState.STOPPED);
    }

    public void setSwerveModules(SwerveModule frontLeftSwerveModule, SwerveModule frontRightSwerveModule,
            SwerveModule backLeftSwerveModule, SwerveModule backRightSwerveModule) {
        this.frontLeftModule = frontLeftSwerveModule;
        this.frontRightModule = frontRightSwerveModule;
        this.backLeftModule = backLeftSwerveModule;
        this.backRightModule = backRightSwerveModule;
    }

    public SwervePathFollower getPathFollower() {
        return pathFollower;
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

    public enum ModuleSnapPositions {
        STRAIGHT(0, 0, 0, 0), DEFENSE(45, 315, 315, 45), NINETY(90, 90, 90, 90);

        private final double frontLeft;
        private final double frontRight;
        private final double backLeft;
        private final double backRight;

        ModuleSnapPositions(double fL, double fR, double bL, double bR) {
            this.frontLeft = fL;
            this.frontRight = fR;
            this.backLeft = bL;
            this.backRight = bR;
        }

        public double[] getSnapPositions() {
            return new double[] { frontLeft, frontRight, backLeft, backRight };
        }
    }

    public enum SwerveControlState {
        STOPPED(false), TELEOP(true), PATH_FOLLOWING(true), BALL_FETCHING(true), SNAPPING(false);

        public boolean rememberLastPosition;

        SwerveControlState(boolean rememberLastPosition) {
            this.rememberLastPosition = rememberLastPosition;
        }
    }

    public enum ModuleLocation {
        FrontLeft, FrontRight, BackLeft, BackRight, TestStandModule
    }
}