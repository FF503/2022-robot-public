package org.frogforce503.robot2022;

import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.Climber.BarStates;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.Hood;
import org.frogforce503.robot2022.subsystems.Hood.HoodStates;
import org.frogforce503.robot2022.subsystems.Intake;
import org.frogforce503.robot2022.subsystems.Intake.IntakeStates;
import org.frogforce503.robot2022.subsystems.JudgeZone;
import org.frogforce503.robot2022.subsystems.JudgeZone.BallColors;
import org.frogforce503.robot2022.subsystems.JudgeZone.ConveyorStates;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.Shooter.ShooterStates;
import org.frogforce503.robot2022.subsystems.Tower;
import org.frogforce503.robot2022.subsystems.Tower.TowerStates;
import org.frogforce503.robot2022.subsystems.Turret;
import org.frogforce503.robot2022.subsystems.Turret.TurretStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.vision.LimelightProcessor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateEngine {

    private static final StateEngine instance = new StateEngine();

    private static RobotStates previousState = RobotStates.DISABLED;
    private static RobotStates curState = RobotStates.DISABLED;
    private boolean driverProfile2 = false;
    private boolean climbSwap = false;
    private boolean presetOverride = false;
    private boolean persistentOverride = false;
    private boolean isAutoEject = false;
    private boolean firstLoop = true;
    private boolean isEjecting = false;
    public int ballCount = 0;
    private int startShotCount = 0;
    private double shootingStartTime = 0;
    public double timeToGetIntoTower = 0;
    private double ejectStartTime = 0;
    private double beamBreakTimeStamp = 0;
    private Debouncer intakingToIdleDebouncer = new Debouncer(.06);
    private boolean hasTwo = false;
    private boolean currentlyShooting = false;
    private double shootingTimestamp;

    private StateEngine() {

    }

    public static StateEngine getInstance() {
        return instance;
    }

    public void disableRobot() {
        curState = RobotStates.DISABLED;
        previousState = RobotStates.DISABLED;
    }

    public RobotStates getRobotState() {
        return curState;
    }

    public void setRobotState(RobotStates s) {
        if (!(s == curState)) {
            previousState = curState;
        }

        curState = s;
    }

    public void setToPreviousState() {
        // if (previousState == RobotStates.PRE_SHOOT || previousState == )
        setRobotState(previousState);
    }

    public void updateState() {
        SmartDashboard.putString("Robot State", curState.toString());
        SmartDashboard.putString("Previous State", previousState.toString());
        SmartDashboard.putNumber("IsEjectingState", isEjecting ? 1 : 0);

        if (curState != RobotStates.IDLE) {
            OI.rumbleDriver(0.0);
        }

        switch (curState) {

            case DISABLED:
                // FIXME: add all stop functions
                Intake.getInstance().setIntakeState(IntakeStates.OFF);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                Tower.getInstance().setTowerState(TowerStates.OFF);
                Shooter.getInstance().setShooterState(ShooterStates.OFF);
                Hood.getInstance().setHoodState(HoodStates.OFF);
                Turret.getInstance().setTurretState(TurretStates.DISABLED);
                DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN);
                break;

            case IDLE:
                Climber.getInstance().setBarStates(BarStates.FLOOR);
                isEjecting = false;
                Intake.getInstance().setIntakeState(IntakeStates.OFF);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                if (!isPresetOverride()) {
                    if (Tower.getInstance().hasBall()) {

                        if (LimelightProcessor.getInstance().onTarget()
                                && LimelightProcessor.getInstance().isTargetVisible()) {
                            OI.rumbleDriver(0.4);
                        } else {
                            OI.rumbleDriver(0.0);
                        }

                        Shooter.getInstance().setShooterState(ShooterStates.VISION);
                        if (/* Turret.getInstance().isTurretOnTarget() */LimelightProcessor.getInstance().onTarget()
                                && LimelightProcessor.getInstance().isTargetVisible()
                                && Shooter.getInstance().isShooterReady()
                                && Hood.getInstance().onTarget()) {

                            if (firstLoop) {
                                DriverFeedback.getInstance().setLEDState(LEDStates.HOT_PINK);
                                firstLoop = false;
                            } else {
                                // DriverFeedback.getInstance().setLEDState(LEDStates.FLASH);
                            }
                        } else {
                            DriverFeedback.getInstance().setLEDState(LEDStates.YELLOW);
                        }
                    } else {
                        Shooter.getInstance().setShooterState(ShooterStates.IDLE);
                        DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN);
                    }
                    Hood.getInstance().setHoodState(HoodStates.VISION);
                }
                Tower.getInstance().setTowerState(TowerStates.OFF);
                break;

            case CLIMBING:
                Intake.getInstance().setIntakeState(IntakeStates.OFF);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                Turret.getInstance().setTurretState(TurretStates.BATTER);
                Shooter.getInstance().setShooterState(ShooterStates.OFF);
                Hood.getInstance().setHoodState(HoodStates.OFF);
                Tower.getInstance().setTowerState(TowerStates.OFF);
                break;
            case MANUAL_FEEDING:
                Tower.getInstance()
                        .setTowerState(Tower.getInstance().hasBall() ? TowerStates.OFF : TowerStates.FEEDING);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.FEEDING);

                if (JudgeZone.getInstance().isBallColorMatching() && JudgeZone.getInstance().hasBall()
                        && Tower.getInstance().hasBall()) {
                    setRobotState(RobotStates.IDLE); // should go to pre-shoot once pre-shoot isconfigured
                    break;
                }

                DriverFeedback.getInstance().updateIntakingColors();

                if (JudgeZone.getInstance().hasBall() && !JudgeZone.getInstance().isBallColorMatching()
                        && JudgeZone.getInstance().getColor() != BallColors.NONE) {
                    beamBreakTimeStamp = 0;
                    setRobotState(RobotStates.EJECT_THROUGH_INTAKE);
                }

                break;
            case MANUAL_EJECTING:
                Tower.getInstance().setTowerState(TowerStates.REVERSE);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.EJECTING);
                break;
            case MANUAL_EJECTING_SLOW:
                Tower.getInstance().setTowerState(TowerStates.REVERSE_SLOW);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.EJECTING_SLOW);
                break;
            case MANUAL_EJECTING_MEDIUM:
                Tower.getInstance().setTowerState(TowerStates.REVERSE_SLOW);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.EJECTING_MEDIUM);
                break;
            case INTAKING:

                // if (JudgeZone.getInstance().hasBall()) {
                // OI.rumbleDriver(0.4);
                // } else {
                // OI.rumbleDriver(0.0);
                // }

                isEjecting = false;
                hasTwo = intakingToIdleDebouncer.calculate(JudgeZone.getInstance().isBallColorMatching()
                        && JudgeZone.getInstance().hasBall()
                        && Tower.getInstance().hasBall());
                if (hasTwo) {
                    firstLoop = true;
                    JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF); // should go to pre-shoot once
                                                                                  // pre-shoot isconfigured

                } else {
                    JudgeZone.getInstance().setConveyorState(ConveyorStates.FEEDING);
                }

                Tower.getInstance()
                        .setTowerState(Tower.getInstance().hasBall() ? TowerStates.OFF : TowerStates.FEEDING);

                Intake.getInstance().setIntakeState(IntakeStates.INTAKING);

                Intake.getInstance().updateBallCount();

                DriverFeedback.getInstance().updateIntakingColors();

                if (JudgeZone.getInstance().hasBall() && !JudgeZone.getInstance().isBallColorMatching()
                        && JudgeZone.getInstance().getColor() != BallColors.NONE) {
                    beamBreakTimeStamp = 0;
                    setRobotState(RobotStates.EJECT_THROUGH_INTAKE);
                    break;
                    // if (Tower.getInstance().hasBall()) {
                    // setRobotState(RobotStates.EJECT_THROUGH_INTAKE);
                    // break;
                    // } else {
                    // setRobotState(RobotStates.EJECT_THROUGH_SHOOTER);
                    // break;
                    // }
                }

                break;
            case AUTON_INTAKING:

                if (JudgeZone.getInstance().hasBall()
                        && Tower.getInstance().hasBall()) {
                    firstLoop = true;
                    setRobotState(RobotStates.IDLE); // should go to pre-shoot once pre-shoot is configured
                    break;
                }

                Tower.getInstance()
                        .setTowerState(Tower.getInstance().hasBall() ? TowerStates.OFF : TowerStates.FEEDING);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.FEEDING);
                Intake.getInstance().setIntakeState(IntakeStates.AUTON_INTAKING);

                // Hood.getInstance().setHoodState(HoodStates.VISION);
                // Shooter.getInstance().setShooterState(ShooterStates.VISION);
                Intake.getInstance().updateBallCount();

                // ADD THIS BACK IF WE WANT TO PICK UP INTAKE WHEN BALL IS THERE

                // Shooter.getInstance()
                // .setShooterState(
                // Turret.getInstance().getCurrentState() == TurretStates.POSITION ?
                // ShooterStates.MANUAL
                // : (Tower.getInstance().hasBall() ? ShooterStates.VISION :
                // ShooterStates.IDLE));

                // Hood.getInstance().setHoodState(
                // Turret.getInstance().getCurrentState() == TurretStates.POSITION ?
                // HoodStates.MANUAL
                // : HoodStates.VISION);

                DriverFeedback.getInstance().updateIntakingColors();
                // if (JudgeZone.getInstance().hasBall() &&
                // !JudgeZone.getInstance().isBallColorMatching()&&
                // JudgeZone.getInstance().getColor() != BallColors.NONE) {
                // beamBreakTimeStamp = 0;
                // setRobotState(RobotStates.EJECT_THROUGH_INTAKE);
                // }

                break;

            case REVERSE_INTAKE:
                Intake.getInstance().setIntakeState(IntakeStates.REVERSE);
                DriverFeedback.getInstance().setLEDState(LEDStates.YELLOW);
                break;

            case PRE_SHOOT:

                // if (LimelightProcessor.getInstance().onTarget()
                // && LimelightProcessor.getInstance().isTargetVisible()) {
                // // DriverFeedback.getInstance().setLEDState(LEDStates.HOT_PINK);
                // OI.rumbleDriver(0.4);
                // // DriverFeedback.getInstance().setLEDState(LEDStates.FLASH);
                // } else {
                // OI.rumbleDriver(0.0);
                // // DriverFeedback.getInstance().setLEDState(LEDStates.YELLOW);
                // }

                if (isPresetOverride()) {
                    if ((Turret.getInstance().getCurrentState() == TurretStates.SHOOTING_PRESET
                            || Turret.getInstance().isTurretOnTarget())
                            && Turret.getInstance().encoderTargetInRange()
                            && Shooter.getInstance().isShooterReady() && Hood.getInstance().onTarget()) {
                        setRobotState(RobotStates.SHOOTING);
                        startShotCount = Shooter.getInstance().getShotCount();
                        shootingStartTime = Timer.getFPGATimestamp();
                        break;
                    }
                    break;
                }
                if ((Shooter.getInstance().getShooterState() == ShooterStates.MANUAL
                        && Shooter.getInstance().isShooterReady() && Turret.getInstance().isTurretOnTarget()
                        && Hood.getInstance().onTarget())) {
                    startShotCount = Shooter.getInstance().getShotCount();
                    shootingStartTime = Timer.getFPGATimestamp();
                    setRobotState(RobotStates.SHOOTING);
                    break;
                }

                if ((Shooter.getInstance().getShooterState() == ShooterStates.VISION
                        && Shooter.getInstance().isShooterReady() && Turret.getInstance().isTurretOnTarget()
                        && Hood.getInstance().onTarget())) {
                    startShotCount = Shooter.getInstance().getShotCount();
                    shootingStartTime = Timer.getFPGATimestamp();
                    setRobotState(RobotStates.SHOOTING);
                    break;
                }

                JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                Tower.getInstance().setTowerState(TowerStates.OFF);
                Shooter.getInstance().setShooterState(ShooterStates.VISION);
                Hood.getInstance().setHoodState(HoodStates.VISION);

                if (Intake.getInstance().getIntakeState() == IntakeStates.INTAKING) {
                    Intake.getInstance().setIntakeState(IntakeStates.DOWN);
                }
                break;
            case AUTON_PRE_SHOOT:

                if ((Shooter.getInstance().getShooterState() == ShooterStates.AUTON_VISION
                        && Shooter.getInstance().isShooterReady() && Turret.getInstance().isTurretOnTarget()
                        && LimelightProcessor.getInstance().onTarget()
                        && Hood.getInstance().onTarget())) {
                    startShotCount = Shooter.getInstance().getShotCount();
                    shootingStartTime = Timer.getFPGATimestamp();
                    setRobotState(RobotStates.AUTON_SHOOTING);
                    break;
                }

                JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                Tower.getInstance().setTowerState(TowerStates.OFF);
                Shooter.getInstance().setShooterState(ShooterStates.AUTON_VISION);
                Hood.getInstance().setHoodState(HoodStates.VISION);

                // if (Intake.getInstance().getIntakeState() == IntakeStates.INTAKING) {
                // Intake.getInstance().setIntakeState(IntakeStates.DOWN);
                // }
                break;

            case SHOOTING:
                DriverFeedback.getInstance().setLEDState(LEDStates.RAINBOW);

                Shooter.getInstance().updateShotCount();

                Tower.getInstance().clearHasBall();
                Tower.getInstance().setTowerState(TowerStates.SHOOT);
                Tower.getInstance().updateShotCount();

                if (!isPresetOverride()) {
                    Hood.getInstance().setHoodState(HoodStates.VISION);
                    Shooter.getInstance().setShooterState(ShooterStates.VISION);
                } else {

                }

                JudgeZone.getInstance().setConveyorState(ConveyorStates.SHOOTING);

                if (Shooter.getInstance().getShotCount() > startShotCount
                        || Timer.getFPGATimestamp() - shootingStartTime >= 3) {
                    setRobotState(RobotState.isAutonomous() ? RobotStates.AUTON_PRE_SHOOT : RobotStates.PRE_SHOOT);
                    break;
                }

                // int elapsedShots = Shooter.getInstance().getShotCount() - startShotCount;
                // if (elapsedShots == 1
                // || Timer.getFPGATimestamp() - shootingStartTime >= 3) {
                // setRobotState(RobotStates.PRE_SHOOT);
                // break;
                // }
                // if (elapsedShots > 1
                // || Timer.getFPGATimestamp() - shootingStartTime >= 3) {
                // setRobotState(RobotStates.IDLE);
                // break;
                // }
                // FIXME: add condition for if the shot count increments then go back to
                // pre-shoot

                break;

            case AUTON_SHOOTING:
                DriverFeedback.getInstance().setLEDState(LEDStates.RAINBOW);

                Shooter.getInstance().updateShotCount();

                Tower.getInstance().clearHasBall();
                Tower.getInstance().setTowerState(TowerStates.SHOOT);
                Tower.getInstance().updateShotCount();

                Hood.getInstance().setHoodState(HoodStates.VISION);
                Shooter.getInstance().setShooterState(ShooterStates.AUTON_VISION);

                JudgeZone.getInstance().setConveyorState(ConveyorStates.SHOOTING);

                if (Shooter.getInstance().getShotCount() > startShotCount
                        || Timer.getFPGATimestamp() - shootingStartTime >= 3) {
                    setRobotState(RobotState.isAutonomous() ? RobotStates.AUTON_PRE_SHOOT : RobotStates.PRE_SHOOT);
                    break;
                }

                // int elapsedShots = Shooter.getInstance().getShotCount() - startShotCount;
                // if (elapsedShots == 1
                // || Timer.getFPGATimestamp() - shootingStartTime >= 3) {
                // setRobotState(RobotStates.PRE_SHOOT);
                // break;
                // }
                // if (elapsedShots > 1
                // || Timer.getFPGATimestamp() - shootingStartTime >= 3) {
                // setRobotState(RobotStates.IDLE);
                // break;
                // }
                // FIXME: add condition for if the shot count increments then go back to
                // pre-shoot

                break;
            case EJECT_THROUGH_INTAKE:
                isEjecting = true;
                DriverFeedback.getInstance().setLEDState(LEDStates.YELLOW);

                Intake.getInstance().setIntakeState(IntakeStates.EJECTING);

                Swerve.getInstance().disableFetcher();

                if (beamBreakTimeStamp == 0) {
                    beamBreakTimeStamp = Timer.getFPGATimestamp();
                }

                if (Timer.getFPGATimestamp() - beamBreakTimeStamp > 0.8) {
                    JudgeZone.getInstance().setConveyorState(ConveyorStates.EJECTING);
                    if (!JudgeZone.getInstance().hasBall()) {
                        if (ejectStartTime == 0) {
                            ejectStartTime = Timer.getFPGATimestamp();
                        }

                        if (Timer.getFPGATimestamp() - ejectStartTime > 0.7) {
                            ejectStartTime = 0;
                            setRobotState(RobotStates.IDLE);
                        }
                    }
                } else {
                    JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                }

                // FIXME: untested, eject was not working when button was pressed with two balls
                // in the robot

                // not exactly sure how this logic is gonna work
                // maybe transition to idle after conveyor beam is broken????

                break;
            case EJECT_THROUGH_SHOOTER:

                Swerve.getInstance().disableFetcher();

                double ejectSpeed = 1500.0;
                Shooter.getInstance().setShooterVelocityManual(ejectSpeed);

                if (Turret.getInstance().isFieldRelativeWithinLimits() || Turret.getInstance().isTurretOnTarget()) {
                    if (beamBreakTimeStamp == 0) {
                        beamBreakTimeStamp = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - beamBreakTimeStamp > 2.0) {
                        setToPreviousState();
                        break;
                    }

                    JudgeZone.getInstance().setConveyorState(ConveyorStates.SHOOTING);
                    Tower.getInstance().setTowerState(TowerStates.SHOOT);
                } else {
                    beamBreakTimeStamp = 0;
                    if (Tower.getInstance().hasBall()) {
                        Tower.getInstance()
                                .setTowerState(TowerStates.OFF);
                        JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                    } else {
                        JudgeZone.getInstance().setConveyorState(ConveyorStates.FEEDING);
                        Tower.getInstance().setTowerState(TowerStates.FEEDING);
                    }
                }

                break;
            case REVERSE_TOWER:
                Intake.getInstance().setIntakeState(IntakeStates.OFF);
                JudgeZone.getInstance().setConveyorState(ConveyorStates.OFF);
                Tower.getInstance().setTowerState(TowerStates.REVERSE);
                Tower.getInstance().clearHasBall();
                break;

            default:
                curState = RobotStates.IDLE;
                break;
        }
    }

    public enum RobotStates {
        DISABLED, IDLE, MANUAL_FEEDING, MANUAL_EJECTING, MANUAL_EJECTING_SLOW, MANUAL_EJECTING_MEDIUM, REVERSE_TOWER,
        REVERSE_INTAKE, FEED_BALL,
        INTAKING, PRE_SHOOT, SHOOTING,
        AUTON_INTAKING, AUTON_PRE_SHOOT, AUTON_SHOOTING, EJECT_THROUGH_INTAKE,
        CLIMBING, EJECT_THROUGH_SHOOTER
    }

    public int getBallCount() {
        return ballCount;
        // FIXME: make this work(actually I think it should work just untested)
    }

    public void setBallCount(int n) {
        ballCount = n;
    }

    public void increaseBallCount() {
        ballCount++;
    }

    public void decreaseBallCount() {
        ballCount--;
    }

    // public void toggleBatterOverride() {
    // batterOverride = !batterOverride;
    // }

    public void enablePersistPreset() {
        persistentOverride = true;
    }

    public void disablePersistPreset() {
        persistentOverride = false;
        Turret.getInstance().moveToTarget();
    }

    public void togglePersistentOverride() {
        if (isPersistPreset()) {
            disablePersistPreset();
            disablePresetOverride();
        } else {
            enablePersistPreset();
        }
    }

    public void enablePresetOverride() {
        presetOverride = true;
    }

    public void disablePresetOverride() {
        presetOverride = false;
        Turret.getInstance().moveToTarget();
    }

    public void togglePresetOverride() {
        if (isPresetOverride()) {
            disablePresetOverride();
        } else {
            enablePresetOverride();
        }
    }

    public void resetShotTimer() {
        shootingTimestamp = Timer.getFPGATimestamp();
        currentlyShooting = true;
    }

    public boolean isPresetOverride() {
        return presetOverride;
    }

    public boolean isPersistPreset() {
        return persistentOverride;
    }

    public boolean toggleDriverControls() {
        if (driverProfile2 == false) {
            driverProfile2 = true;
        } else {
            driverProfile2 = false;
        }
        return driverProfile2;
    }

    public boolean getDriverControls() {
        return driverProfile2;
    }

    public boolean toggleClimbingSwap() {
        if (climbSwap == false) {
            climbSwap = true;
            Climber.getInstance().toggleTeleOpControl();
        } else {
            climbSwap = false;
            Climber.getInstance().toggleTeleOpControl();

        }
        return climbSwap;
    }

    public boolean getClimbingSwap() {
        return climbSwap;
    }

}
