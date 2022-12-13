/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2022;

import java.util.Arrays;

import org.frogforce503.robot2022.RobotState.GameState;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.Hood;
import org.frogforce503.robot2022.subsystems.Intake;
import org.frogforce503.robot2022.subsystems.JudgeZone;
import org.frogforce503.robot2022.subsystems.JudgeZone.BallColors;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.Tower;
import org.frogforce503.robot2022.subsystems.Turret;
import org.frogforce503.robot2022.subsystems.Turret.HomePositions;
import org.frogforce503.robot2022.subsystems.Turret.TurretStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.ModuleSnapPositions;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.SwerveControlState;
import org.frogforce503.robot2022.subsystems.vision.LimelightProcessor;
import org.frogforce503.robot2022.tests.RobotTest;
import org.frogforce503.robot2022.tests.TestInput;
import org.frogforce503.robot2022.tests.TestManager;
import org.frogforce503.robot2022.tests.modes.ReZeroSwerveRoutine;
import org.frogforce503.robot2022.tests.modes.SwerveSpin;
import org.frogforce503.robot2022.tests.modes.SwerveStep;
import org.frogforce503.robot2022.tests.modes.TurretReZero;
import org.json.simple.JSONObject;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

    private String allianceColor = "";

    public static RobotHardware bot;
    Compressor c;

    private double mDisabledTime = 0.0;
    boolean testEnabledDS = false;

    // StringLogEntry myStringLog;

    NetworkTableInstance networkTableInstance;
    NetworkTable testTable, FMSTable;
    SendableChooser<String> testChooser;
    SendableChooser<String> allianceChooser;
    TestManager testManager = new TestManager(new SwerveSpin(), new SwerveStep(), new ReZeroSwerveRoutine(),
            new TurretReZero());

    public Robot() {
        addPeriodic(() -> {
            LimelightProcessor.getInstance().forceMagnified(false);
            LimelightProcessor.getInstance().getHubError();
            Turret.getInstance().readPeriodicInputs();
            Turret.getInstance().writePeriodicOutputs();
            Turret.getInstance().outputTelemetry();
            // Turret.getInstance().updatePIDValues();
        }, 0.010, 0.010);
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    @SuppressWarnings("unchecked")
    public void robotInit() {
        // Initialize bot
        // DataLogManager.start();

        // DataLog log = DataLogManager.getLog();
        // myStringLog = new StringLogEntry(log, "/my/string");
        RobotState.getInstance().setCurrentRobot(RobotState.Bot.CompBot);
        bot = RobotHardware.getInstance();

        OI.initialize();

        // // Initialize Subsystems
        Swerve.getInstance().zeroSensors();
        Swerve.getInstance().enableBrakeMode(false);

        c = new Compressor(0, Constants.PCMType);

        FMSTable = NetworkTableInstance.getDefault().getTable("FMSInfo");

        RobotState.getInstance().setAllianceColor(
                FMSTable.getEntry("IsRedAlliance").getBoolean(true) ? BallColors.RED : BallColors.BLUE);

        // // .addAuto("Terminal 5 Ball", new Terminal5Ball().getGroup())
        // // .addAuto("Straight and 2", new StraightAnd2().getGroup())
        // .addAuto("Right 3 Ball", new Right3Ball().getGroup())
        // .addAuto("Right 4 Ball", new Right4Ball().getGroup())
        // .addAuto("Action Test Auto", new ActionTestAuto().getGroup());

        LiveWindow.disableAllTelemetry();

        Intake.getInstance().initTelemetry();
        JudgeZone.getInstance().initTelemetry();
        Tower.getInstance().initTelemetry();
        Shooter.getInstance().initTelemetry();
        Hood.getInstance().initTelemetry();
        Turret.getInstance().initTelemetry();
        DriverFeedback.getInstance().initTelemetry();
        // Swerve.getInstance().initTelemetry();

        if (Robot.bot.hasClimber) {
            Climber.getInstance().initTelemetry();
        }
        // // initialize everything

        LimelightProcessor.getInstance();
        // // autoChooser.resetClass();

        // send test info
        networkTableInstance = NetworkTableInstance.getDefault();
        testTable = networkTableInstance.getTable("testModes");

        testTable.getEntry("testList").setStringArray(testManager.getTestList());

        // set default test info
        testTable.getEntry("enabled").setBoolean(false);
        if (!Arrays.asList(testManager.getTestList()).contains(testTable.getEntry(
                "selectedTest").getString("")))
            testTable.getEntry("selectedTest").setString(testManager.getDefault());

        // send test input info
        JSONObject testsObject = new JSONObject();

        for (RobotTest test : testManager.getTests()) {
            TestInput<Object>[] inputs = test.getInputs();
            JSONObject inputsObject = new JSONObject();

            for (TestInput<Object> input : inputs) {
                inputsObject.put(input.name, input.getType());
            }

            if (inputs.length > 0)
                testsObject.put(test.getName(), inputsObject);
        }

        testTable.getEntry("test_input_info").setString(testsObject.toJSONString());

        SmartDashboard.putData(CommandScheduler.getInstance());

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        RobotState.getInstance().setAllianceColor(
                FMSTable.getEntry("IsRedAlliance").getBoolean(true) ? BallColors.RED : BallColors.BLUE);
        CommandScheduler.getInstance().run();
        JudgeZone.getInstance().updateBallColors();
        Climber.getInstance().outputTelemetry();
        SmartDashboard.putBoolean("Test Enabled (DS)", testEnabledDS);
        Turret.getInstance().readPeriodicInputs();
        Turret.getInstance().outputTelemetry();
    }

    @Override
    public void disabledInit() {
        RobotState.getInstance().setGameState(RobotState.GameState.DISABLED);
        Shooter.getInstance().updatePreferences();
        CommandScheduler.getInstance().cancelAll();
        DriverFeedback.getInstance().setLEDState(LEDStates.OFF);

        Swerve.getInstance().stop();
        Swerve.getInstance().snapModulesTo(ModuleSnapPositions.STRAIGHT);

        mDisabledTime = Timer.getFPGATimestamp();
        testEnabledDS = false;
    }

    @Override
    public void disabledPeriodic() {
        StateEngine.getInstance().setRobotState(RobotStates.DISABLED);
        StateEngine.getInstance().updateState();
        DriverFeedback.getInstance().setLEDState(LEDStates.OFF);

        // FIXME: Disable things here

        // FIXME:Reimplement this
        // Wait for two seconds to brake then put drive in coast mode once
        // if (Timer.getFPGATimestamp() - mDisabledTime >= 2 &&
        // Swerve.getInstance().getBrakeMode()) {
        // Swerve.getInstance().enableBrakeMode(false);
        // }
    }

    @Override
    public void autonomousInit() {
        RobotState.getInstance().setGameState(RobotState.GameState.AUTON);
        // if unnecessary. Defaulting to Red
        // Alliance for AUTON
        // RobotState.getInstance().setAllianceColor(allianceChooser.getSelected());
        Swerve.getInstance().enableBrakeMode(true);
        Swerve.getInstance().snapModulesTo(ModuleSnapPositions.STRAIGHT);

        Turret.getInstance().setHomePosition(HomePositions.RIGHT);
        Turret.getInstance().setTurretState(TurretStates.VISION_MOVING);

        LimelightProcessor.getInstance().forceMagnified(false);

        Shooter.getInstance().resetShotCount();

        // autoChooser.setAndInitAuto("Right 4 Ball");

        StateEngine.getInstance().setRobotState(RobotStates.IDLE);

        // Shuffleboard.startRecording();
        c.enableDigital();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Swerve.getInstance().writePeriodicOutputs();

        Intake.getInstance().writePeriodicOutputs();
        Intake.getInstance().outputTelemetry();

        JudgeZone.getInstance().writePeriodicOutputs();
        JudgeZone.getInstance().updateBallColors();
        JudgeZone.getInstance().outputTelemetry();

        Tower.getInstance().writePeriodicOutputs();
        Tower.getInstance().outputTelemetry();

        Turret.getInstance().readPeriodicInputs();
        Turret.getInstance().writePeriodicOutputs();
        Turret.getInstance().outputTelemetry();
        // Turret.getInstance().updatePIDValues();

        Hood.getInstance().writePeriodicOutputs();
        Hood.getInstance().outputTelemetry();

        Shooter.getInstance().writePeriodicOutputs();
        Shooter.getInstance().outputTelemetry();

        StateEngine.getInstance().updateState();
    }

    @Override
    public void teleopInit() {
        // autoChooser.endAuto();
        StateEngine.getInstance().disableRobot();

        RobotState.getInstance().setGameState(RobotState.GameState.TELEOP);

        // Initialize swerve stabilization to avoid sudden undesired movement
        // Swerve.getInstance().initializeSwerveStabilization();
        Swerve.getInstance().setControlState(SwerveControlState.TELEOP);
        Swerve.getInstance().snapModulesTo(ModuleSnapPositions.STRAIGHT);
        Swerve.getInstance().enableBrakeMode(true);
        Swerve.getInstance().resetStabilizationHeading();

        LimelightProcessor.getInstance().forceMagnified(false);

        // Turret.getInstance().setHomePosition(HomePositions.RIGHT);
        Turret.getInstance().setTurretState(TurretStates.VISION_MOVING);
        // Turret.getInstance().updatePIDValues();

        StateEngine.getInstance().setRobotState(RobotStates.IDLE);
        OI.flushJoystickCaches();
        // LEDController.getInstance().setLEDState(LEDStates.DARK_GREEN);
        c.enableDigital();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        if (Robot.bot.hasClimber) {
            Climber.getInstance().writePeriodicOutputs();
            // Climber.getInstance().outputTelemetry();
        }

        // myStringLog.append("hello mom");
        if (!StateEngine.getInstance().getClimbingSwap()) {
            Swerve.getInstance().writePeriodicOutputs();
        }
        Swerve.getInstance().outputTelemetry();
        // Swerve.getInstance().updatePIDValues();

        Intake.getInstance().writePeriodicOutputs();
        Intake.getInstance().outputTelemetry();
        // Intake.getInstance().updatePIDValues();

        JudgeZone.getInstance().updateBallColors();
        JudgeZone.getInstance().writePeriodicOutputs();
        JudgeZone.getInstance().outputTelemetry();
        // JudgeZone.getInstance().updatePIDValues();

        Tower.getInstance().writePeriodicOutputs();
        Tower.getInstance().outputTelemetry();
        // Tower.getInstance().updatePIDValues();

        // Turret.getInstance().readPeriodicInputs();
        // Turret.getInstance().writePeriodicOutputs();
        // Turret.getInstance().outputTelemetry();
        // Turret.getInstance().updatePIDValues();

        Hood.getInstance().writePeriodicOutputs();
        // Hood.getInstance().setHoodAngle(155);
        Hood.getInstance().outputTelemetry();

        Shooter.getInstance().writePeriodicOutputs();
        Shooter.getInstance().outputTelemetry();
        // Shooter.getInstance().updatePIDValues();

        DriverFeedback.getInstance().writePeriodicOutputs();
        DriverFeedback.getInstance().outputTelemetry();

        StateEngine.getInstance().updateState();
    }

    @Override
    public void testInit() {
        RobotState.getInstance().setGameState(RobotState.GameState.TEST);
        testEnabledDS = true;

        // networkTableInstance = NetworkTableInstance.getDefault(); // now done in
        // // robot init
        // testTable = networkTableInstance.getTable("testModes"); // now done in robot
        // init

        testChooser = new SendableChooser<>();

        boolean firstDone = false;

        // tells dashboard what each test's inputs are and what type the input is,
        // example output:
        /**
         * { "test1": { "power": "Double" }, "test2": { "open": "Boolean", "mode":
         * "String" } }
         */

        for (String testName : testManager.getTestList()) {
            if (!firstDone)
                testChooser.setDefaultOption(testName, testName);
            else
                testChooser.addOption(testName, testName);
            firstDone = true;
        }

        SmartDashboard.putData(testChooser);

        boolean testEnabled = false;
        SmartDashboard.putBoolean("testEnabled", testEnabled);

        testTable.getEntry("testList").setStringArray(testManager.getTestList()); //
        // now done in robot init

        TestManager.clearLog();

        // Swerve.getInstance().stop();

        // networkTableInstance.startClientTeam(502);

        RobotState.getInstance().setGameState(GameState.TEST);

        // c.enableDigital();
    }

    /**
     * This function is called periodically during test mode.
     */
    String testOutput = "Test Disabled";

    boolean testWasEnabled = false;

    @Override
    public void testPeriodic() {

        // testManager.setActive(testTable.getEntry("selectedTest").getString(
        // testManager.getDefault()));

        testTable.getEntry("selectedTest").setString(testManager.checkActiveTest());

        // TestInput<Object>[] n_inputs = testManager.getActive().getInputs();

        // testManager.setActive(testChooser.getSelected());

        boolean testEnabled = testTable.getEntry("enabled").getBoolean(false);
        // boolean testEnabled = SmartDashboard.getBoolean("testEnabled", false);
        // boolean testEnabled = false;

        if (testEnabled) {
            testManager.handleInputTable(testTable);

            if (testWasEnabled)
                testManager.init();

            // testManager.handleInput();
            testManager.run();
            String output = testManager.getOutput();
            if (!output.equals(testOutput))
                testOutput = output;

            // testManager.getActiveSubsystem().outputTelemetry();
            testWasEnabled = false;
        } else {
            if (!testOutput.equals("Test Disabled")) {
                testOutput = "Test Disabled";
                testManager.stop();
            }
            testWasEnabled = true;
        }

        testTable.getEntry("testOutput").setString(testOutput);
        testTable.getEntry("testLog").setString(testManager.getLog());
        SmartDashboard.putString("testOutput", testOutput);
    }
}
