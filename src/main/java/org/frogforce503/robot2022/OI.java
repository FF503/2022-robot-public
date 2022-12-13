package org.frogforce503.robot2022;

import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.commands.AutoSnapCommand;
import org.frogforce503.robot2022.commands.NoopCommand;
import org.frogforce503.robot2022.commands.RobotStateCommand;
import org.frogforce503.robot2022.commands.DriverCommands.DriverACommand;
import org.frogforce503.robot2022.commands.DriverCommands.DriverBPressedCommand;
import org.frogforce503.robot2022.commands.DriverCommands.DriverBReleasedCommand;
import org.frogforce503.robot2022.commands.DriverCommands.DriverRBCommand;
import org.frogforce503.robot2022.commands.DriverCommands.OperatorDPadLeftReleased;
import org.frogforce503.robot2022.commands.DriverCommands.OperatorDPadRightReleased;
import org.frogforce503.robot2022.commands.DriverCommands.OperatorDPadUpReleased;
import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.JudgeZone;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

@SuppressWarnings("unused")
public class OI {

    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());

    private static final XboxController driverJoystick = new XboxController(0);
    private static final XboxController operatorJoystick = new XboxController(1);

    private static final JoystickButton driverA = new JoystickButton(driverJoystick, 1);
    private static final JoystickButton driverB = new JoystickButton(driverJoystick, 2);
    private static final JoystickButton driverX = new JoystickButton(driverJoystick, 3);
    private static final JoystickButton driverY = new JoystickButton(driverJoystick, 4);

    private static final JoystickButton driverRB = new JoystickButton(driverJoystick, 6);
    private static final JoystickButton driverLB = new JoystickButton(driverJoystick, 5);

    private static final Trigger driverLT = new Trigger(OI::getDriverLeftTrigger);
    private static final Trigger driverRT = new Trigger(OI::getDriverRightTrigger);

    private static final Trigger driverPOVUp = new Trigger(OI::getDriverPOVForward);
    private static final Trigger driverPOVDown = new Trigger(OI::getDriverPOVReverse);
    private static final Trigger driverPOVLeft = new Trigger(OI::getDriverPOVLeft);
    private static final Trigger driverPOVRight = new Trigger(OI::getDriverPOVRight);

    private static final JoystickButton driverSelect = new JoystickButton(driverJoystick, 7);
    private static final JoystickButton driverMenu = new JoystickButton(driverJoystick, 8);

    private static final JoystickButton operatorY = new JoystickButton(operatorJoystick, 4);
    private static final JoystickButton operatorX = new JoystickButton(operatorJoystick, 3);
    private static final JoystickButton operatorB = new JoystickButton(operatorJoystick, 2);
    private static final JoystickButton operatorA = new JoystickButton(operatorJoystick, 1);
    private static final JoystickButton operatorMenu = new JoystickButton(operatorJoystick, 8);
    private static final JoystickButton operatorSelect = new JoystickButton(operatorJoystick, 7);
    private static final JoystickButton operatorLB = new JoystickButton(operatorJoystick, 5);
    private static final JoystickButton operatorRB = new JoystickButton(operatorJoystick, 6);
    private static final JoystickButton operatorRJ = new JoystickButton(operatorJoystick, 10);

    private static final Trigger operatorLT = new Trigger(OI::getOperatorLeftTrigger);
    private static final Trigger operatorRT = new Trigger(OI::getOperatorRightTrigger);

    private static final Trigger operatorPOVUp = new Trigger(OI::getOperatorPOVForward);
    private static final Trigger operatorPOVLeft = new Trigger(OI::getOperatorPOVLeft);
    private static final Trigger operatorPOVDown = new Trigger(OI::getOperatorPOVReverse);
    private static final Trigger operatorPOVRight = new Trigger(OI::getOperatorPOVRight);

    public static void initialize() {

        // @formatter:off

        /* DRIVER CONTROLS */
        driverPOVUp.whenInactive(() -> Swerve.getInstance().toggleRobotCentric());
        driverMenu.whenReleased(()-> StateEngine.getInstance().toggleDriverControls());
        driverLB.whenReleased(() -> Swerve.getInstance().toggleSlowMode());
        driverSelect.whenReleased(() -> StateEngine.getInstance().toggleClimbingSwap());
        // driverLB.whenReleased(new FetchBallCommand(0.75)); // FIXME: set back to slowmode
        
        driverRB.whenReleased(new DriverRBCommand());
      
        driverRT.whenInactive(new RobotStateCommand(RobotStates.IDLE));
        driverRT.whenActive(new ConditionalCommand(new RobotStateCommand(RobotStates.PRE_SHOOT), new NoopCommand(), () -> StateEngine.getInstance().getRobotState() != RobotStates.SHOOTING));
        driverRT.whenInactive(new ConditionalCommand(new InstantCommand(()->StateEngine.getInstance().disablePresetOverride()), new NoopCommand(), () -> !StateEngine.getInstance().isPersistPreset()));

        driverA.whenReleased(new DriverACommand());
        // driverA.whenActive(new InstantCommand(()->Turret.getInstance().setOpenLoop(1.0)));
        // driverA.whenReleased(new InstantCommand(()->Turret.getInstance().setOpenLoop(0.0)));
        // driverB.whenActive(new InstantCommand(()->Turret.getInstance().setOpenLoop(-1.0)));
        // driverB.whenReleased(new InstantCommand(()->Turret.getInstance().setOpenLoop(0.0)));
        driverB.whenPressed(new DriverBPressedCommand()); // reset gyro to straight
        driverB.whenReleased(new DriverBReleasedCommand());
        driverX.whenReleased(new AutoSnapCommand(0));
        // driverX.whenActive(new DriverXPressedCommand());
        // driverX.whenInactive(new DriverXReleasedCommand());
        // driverY.whenReleased(() -> Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE));
        driverY.whenActive(new InstantCommand(() -> StateEngine.getInstance().setRobotState(RobotStates.REVERSE_TOWER)));
        driverY.whenInactive(new InstantCommand(() -> StateEngine.getInstance().setToPreviousState()));

        // driverPOVUp.whenActive(new InstantCommand(() -> Swerve.getInstance().resetOdometry(new Pose2d(5.779, Units.feetToMeters((26 + 7 / 12)) - 3.9838, Rotation2d.fromDegrees(0))))); // the feet subtraction is to account for the y axis offset in the path planner
        // driverPOVUp.whenActive(new InstantCommand(() -> Swerve.getInstance().resetOdometry(new Pose2d(10.4659, Units.feetToMeters((26 + 7 / 12)) - 3.9838, Rotation2d.fromDegrees(180))))); // the feet subtraction is to account for the y axis offset in the path planner

        driverPOVDown.whenInactive(new AutoSnapCommand(0));
        // driverX.whenReleased(() -> Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE)); // go into defense mode
    

        
        /* OPERATOR CONTROLS */
          //REGULAR OPERATOR CONTROLS
        operatorA.whenPressed(new ConditionalCommand(new RobotStateCommand(RobotStates.INTAKING), new RobotStateCommand(RobotStates.IDLE),()-> StateEngine.getInstance().getRobotState() != RobotStates.INTAKING));
        operatorA.whenReleased(new ConditionalCommand(new RobotStateCommand(RobotStates.IDLE), new NoopCommand(),()-> StateEngine.getInstance().getRobotState() == RobotStates.INTAKING));

        operatorB.whileHeld(new ConditionalCommand(new RobotStateCommand(RobotStates.REVERSE_INTAKE), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
        operatorB.whenReleased(new ConditionalCommand(new InstantCommand(()-> StateEngine.getInstance().setToPreviousState()), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
       
        operatorX.whenReleased(new InstantCommand(()-> JudgeZone.getInstance().togglePixyCam()));
        // operatorX.whileHeld(new ConditionalCommand(new RobotStateCommand(RobotStates.EJECT_THROUGH_INTAKE), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
        // operatorX.whenReleased(new ConditionalCommand(new InstantCommand(()-> StateEngine.getInstance().setToPreviousState()), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
       
        operatorLT.whileActiveContinuous(new ConditionalCommand(new RobotStateCommand(RobotStates.MANUAL_EJECTING), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
        operatorLT.whenInactive(new ConditionalCommand(new InstantCommand(()-> StateEngine.getInstance().setToPreviousState()), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
        operatorRT.whileActiveContinuous(new ConditionalCommand(new RobotStateCommand(RobotStates.MANUAL_FEEDING), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
        operatorRT.whenInactive(new ConditionalCommand(new InstantCommand(()-> StateEngine.getInstance().setToPreviousState()), new NoopCommand(), ()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING));
   
        operatorPOVDown.whenInactive(new InstantCommand(()-> StateEngine.getInstance().disablePresetOverride()));
        operatorPOVUp.whenInactive(new OperatorDPadUpReleased());
        operatorPOVLeft.whenInactive(new OperatorDPadLeftReleased());
        operatorPOVRight.whenInactive(new OperatorDPadRightReleased());
       
        
       //CLIMBING CONTROLS
        operatorSelect.whenReleased(new ConditionalCommand(new RobotStateCommand(RobotStates.CLIMBING), new RobotStateCommand(RobotStates.IDLE),()-> StateEngine.getInstance().getRobotState() != RobotStates.CLIMBING)); // ENABLE CLIMBING AND DISENGAGE ALL OTHER SUBSYSTEMS
        operatorY.whenReleased(new InstantCommand(()-> Climber.getInstance().handleClimberStates())); // AUTOMATED HANDLE MOVE TO NEXT BAR
        operatorLB.whenReleased(()-> Climber.getInstance().rezeroClimber()); //MANUAL REZERO
        operatorRB.whenReleased(new ConditionalCommand(new InstantCommand(() -> Climber.getInstance().toggleRotate()), new RobotStateCommand(RobotStates.IDLE),()-> StateEngine.getInstance().getRobotState() == RobotStates.CLIMBING));//MANUAL ROTATE
        operatorMenu.whenReleased(new InstantCommand(() -> Climber.getInstance().toggleTeleOpControl())); //MANUAL ARM MOVEMENT
        
        // @formatter:on

    }

    public static void rumbleDriver(double r) {
        driverJoystick.setRumble(RumbleType.kLeftRumble, r);
        driverJoystick.setRumble(RumbleType.kRightRumble, r);
    }

    public static JoystickButton getToggleIntakeButton() {
        return driverA;
    }

    public static JoystickButton getReverseIntakeButton() {
        return driverB;
    }

    public static JoystickButton getPreShootButton() {
        return driverX;
    }

    public static JoystickButton getBatterOverrideButton() {
        return driverSelect;
    }

    // -----

    public static boolean tryingToDrive() {
        return (Math.abs(Math.hypot(getDriverLeftXValue(), getDriverLeftYValue())) >= Swerve.JOYSTICK_DRIVE_TOLERANCE)
                || (Math.abs(getDriverRightXValue()) > Swerve.JOYSTICK_TURN_TOLERANCE);
    }

    public static boolean getAnchorButton() {
        return getDriverLeftTrigger();
    }

    public static boolean getClimbOverride() {
        return operatorMenu.get();
    }

    public static boolean getDriverXButton() {
        return driverX.get();
    }

    public static boolean getDriverLeftBumper() {
        return driverLB.get();
    }

    public static boolean getDriverRightBumper() {
        return driverRB.get();
    }

    public static boolean getZeroGyroButton() {
        return driverJoystick.getBButton();
    }

    public static boolean stopShooterButtonReleased() {
        return operatorJoystick.getXButtonReleased();
    }

    public static boolean shortShotButtonReleased() {
        return operatorJoystick.getYButtonReleased();
    }

    public static boolean longShotButtonReleased() {
        return operatorJoystick.getAButtonReleased();
    }

    public static boolean toggleSpindexerButtonReleased() {
        return driverJoystick.getBButtonReleased();
    }

    public static boolean toggleIntakeButtonReleased() {
        return operatorJoystick.getRightBumperReleased();
    }

    public static boolean toggleEjectButtonReleased() {
        return operatorJoystick.getLeftBumperReleased();
    }

    public static boolean toggleIdleModeButtonReleased() {
        return operatorJoystick.getBButtonReleased();
    }

    public static boolean spindexerHomeButtonReleased() {
        return operatorJoystick.getBackButtonReleased();
    }

    public static boolean aimButtonPressed() {
        return driverRB.get();
    }

    public static boolean strafeButtonPressed() {
        return driverLB.get();
    }

    public static boolean getAimButtonPressed() {
        return driverJoystick.getRightBumperPressed();
    }

    public static boolean getStrafeButtonPressed() {
        return driverJoystick.getLeftBumperPressed();
    }

    public static boolean getAimButtonReleased() {
        return driverJoystick.getRightBumperReleased();
    }

    public static boolean getStrafeButtonReleased() {
        return driverJoystick.getLeftBumperReleased();
    }

    public static boolean getOverrideShootButtonReleased() {
        return driverJoystick.getBackButtonReleased();
    }

    public static double getDriverLeftYValue() {
        return driverJoystick.getRawAxis(1);
    }

    public static double getDriverLeftXValue() {
        return driverJoystick.getRawAxis(0);
    }

    public static double getDriverRightYValue() {
        return driverJoystick.getRawAxis(5);
    }

    public static double getDriverRightXValue() {
        return driverJoystick.getRawAxis(4);
    }

    public static boolean getClimbSnapButton() {
        return driverJoystick.getStartButton();
    }

    public static int getDriverPOV() {
        return driverJoystick.getPOV();
    }

    public static boolean getDriverPOVForward() {
        return getDriverPOV() == 0;
    }

    public static boolean getDriverPOVReverse() {
        return getDriverPOV() == 180;
    }

    public static boolean getDriverPOVLeft() {
        return getDriverPOV() == 270;
    }

    public static boolean getDriverPOVRight() {
        return getDriverPOV() == 90;
    }

    public static int getOperatorPOV() {
        return operatorJoystick.getPOV();
    }

    public static boolean getOperatorPOVForward() {
        return getOperatorPOV() == 0;
    }

    public static boolean getOperatorPOVLeft() {
        return getOperatorPOV() == 270;
    }

    public static boolean getOperatorPOVRight() {
        return getOperatorPOV() == 90;
    }

    public static boolean getOperatorPOVReverse() {
        return getOperatorPOV() == 180;
    }

    public static boolean getDriverLeftTrigger() {
        return driverJoystick.getRawAxis(2) >= 0.5;
    }

    public static boolean getDriverRightTrigger() {
        return driverJoystick.getRawAxis(3) >= 0.5;
    }

    public static double getOperatorLeftYValue() {
        return operatorJoystick.getRawAxis(1);
    }

    public static double getOperatorRightYValue() {
        return operatorJoystick.getRawAxis(5);
    }

    public static boolean getOperatorLeftTrigger() {
        return operatorJoystick.getRawAxis(2) >= 0.5;
    }

    public static boolean getOperatorRightTrigger() {
        return operatorJoystick.getRawAxis(3) >= 0.5;
    }

    public static boolean getSnapToZeroButton() {
        return driverJoystick.getAButton();
    }

    public static boolean shooterIncrementPressed() {
        return getOperatorRightTrigger() && getOperatorPOV() == 0;
    }

    public static boolean shooterDecrementPressed() {
        return getOperatorRightTrigger() && getOperatorPOV() == 180;
    }

    public static JoystickButton getAimButton() {
        return driverRB;
    }

    public static boolean getShootButtonPressed() {
        return driverJoystick.getLeftBumperPressed();
    }

    public static boolean getShootButtonReleased() {
        return driverJoystick.getLeftBumperReleased();
    }

    public static void setDriveRumble(double rumble) {
        driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
        driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
    }

    public static boolean getOperatorXButton() {
        return operatorJoystick.getXButtonPressed();
    }

    public static boolean getOperatorYButton() {
        return operatorJoystick.getYButtonPressed();
    }

    public static boolean getOperatorAButton() {
        return operatorJoystick.getAButtonPressed();
    }

    public static void flushJoystickCaches() {
        for (int button = 1; button <= 10; button++) {
            driverJoystick.getRawButtonPressed(button);
            driverJoystick.getRawButtonReleased(button);
            operatorJoystick.getRawButtonPressed(button);
            operatorJoystick.getRawButtonReleased(button);
        }
    }

}
