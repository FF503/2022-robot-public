/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2022;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2022.RobotState.Bot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Add your docs here.
 */
public abstract class RobotHardware {

    private static RobotHardware instance = null;

    // Constants
    // SwerveFileNames
    public String backLeftName;
    public String backRightName;
    public String frontLeftName;
    public String frontRightName;

    // Climber

    public int pigeonID;

    public boolean hasClimber;

    public int climberID;
    public int climberReverseID;
    public int climberForwardID;
    public int fixedLeftSwitchID;
    public int fixedRightSwitchID;
    public int extendLeftSwitchID;
    public int extendRightSwitchID;
    public int climberZeroSwitchID;
    public int climbLeftMotorID;
    public int climbRightMotorID;

    public int leftClimbZeroID;
    public int rightClimbZeroID;
    public int leftFixedID;
    public int rightFixedID;

    // Conveyor
    public boolean hasPixyCam;
    public int conveyorID;
    public int sorterSoldenoidID;// FIXME might need to remove
    public int conveyorBeamID;
    public double kConveyorNeoKP;
    public double kConveyorNeoKI;
    public double kConveyorNeoKD;
    public double kConveyorNeoKS;
    public double kConveyorNeoKV;
    public double kConveyorNeoKA;
    public double kConveyorNeoIZone;
    public double kConveyorRevsPerMotor;
    public double kConveyorVelocity;
    public double kConveyorIntakingVelocity;

    // Shooter
    public int shooterID;
    public int jzBeamID;
    public int shooterMasterID;
    public int shooterFollowerID;
    public double kShooterRevsPerMotor;
    public double kShooterDiameter;
    public int FEET_25_RPM; // 22 deg
    public int FEET_10_RPM; // 35 deg
    // Shooter PID (F, SVA)
    public double kShooterFalconKP;
    public double kShooterFalconKI;
    public double kShooterFalconKD;
    public double kShooterFalconKS;
    public double kShooterFalconKV;
    public double kShooterFalconKA;
    public double kShooterFalconIZone;
    // Shooter RPMs
    public double kShooterIdle;
    public double kShooterBatterRPM;

    // Tower
    public double kTowerRevsPerMotor;
    public double kTowerNeoKP;
    public double kTowerNeoKD;
    public double kTowerNeoKV;
    public double kSHooterFalconkFF;
    public double kTowerNeoKS;
    public double kTowerFeedingVel;
    public double kTowerShootingVel;
    public int towerID;
    public int towerBeamID;

    // Intake
    public int intakeID;
    public int shiftForwardID;
    public int shiftReverseID;
    public double kIntakeNeoKP;
    public double kIntakeNeoKI;
    public double kIntakeNeoKD;
    public double kIntakeNeoKS;
    public double kIntakeNeoKV;
    public double kIntakeNeoKA;
    public double kIntakeNeoIZone;
    public double kIntakeRevsPerMotor;
    public double kIntakeVelocity;
    public double kAutoIntakeVelocity;
    public int kIntakeCurrentLimit;

    // Turret
    public int turretID;
    public boolean isTurretInverted;
    public boolean turretHasGadgeteerLimits;
    public int kTurretMaxSpeed;
    public int kTurretAngleTolerance;
    public int kTurretStartingAngle;
    public double kTurretKP;
    public double kTurretKI;
    public double kTurretKD;
    public double kTurretKFF;
    public double kTurretIzone;
    public double kTurretCruiseV;
    public double kTurretMA;
    public double kTurretMaxCWLimit;
    public double kTurretMaxCCWEncLimit;
    public double kTurretMaxCWEncLimit;
    public double kTurretMaxCCWLimit;

    public double kTurretLockedKP;
    public double kTurretLockedKI;
    public double kTurretLockedKD;
    public double kTurretLockedKFF;
    public double kTurretLockedIzone;

    public double kTurretSnapKP;
    public double kTurretSnapKI;
    public double kTurretSnapKD;
    public double kTurretSnapKFF;
    public double kTurretSnapIzone;

    public int kTurretCwLimitID;
    public int kTurretCCwLimitID;

    public double kTurretEncoderFrontOffset;
    public double kTurretEncoderBatterOffset;
    public double kTurretEncoderHomeOffset;
    public double kTurretEncoderZeroOffset;

    public double kTurretWideKP;
    public double kTurretWideKI;
    public double kTurretWideKD;
    public double kTurretWideKFF;

    // Hood
    public int leftLinearActuatorID;
    public int rightLinearActuatorID;
    public double kHoodSpeed;
    public double kHoodHorizontalOffset = 24;
    public int hoodID;

    // LED PWM PORT

    // Panel Spinner
    public int panelSpinnerID;
    public int panelSpinnerSolenoidID;
    // Limelight Constants
    public double visionAreaConstant = 1.0;
    public double yVisionkP = 0.3;
    public double xVisionkP = 0.8;
    public double limelightVerticalAngle;
    public double limelightFloorClearance;
    public double limelightMagnifiedOffset;
    public double magnifyThreshold = 0.9;
    public double strafeTXTolerance = 1.5;// 0.25;
    public double limelightPopTime = 0.5;
    // Shooter Constants
    public double visionTargetingHeadingKp;
    public double visionTargetingHeadingMinPower;
    public double visionTargetingHeadingTxTolerance;
    public double visionTargetingStrafeKp;
    public double visionTargetingStrafeMinPower;
    public double visionTargetingStrafeTxTolerance;
    public double visionTargetingRangeKp;
    public double visionTargetingRangeMinPower;
    public double visionTargetingRangeTolerance;
    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;
    public double wheelDiameter;
    // Hood Constants
    public double kMinHoodAngle;
    public double kMaxHoodAngle;
    public double kBatterHoodAngle;

    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToFrontRight;
    public Translation2d kVehicleToBackRight;
    public Translation2d kVehicleToFrontLeft;
    public Translation2d kVehicleToBackLeft;
    public Translation2d[] kModulePositions;

    public double limelightTXTolerance;

    // LED PWM PORT
    public int LED_ID = 1;

    public double[][] shotMapValues, shotMapNewBalls;

    public String limelightURL;
    public String usbCameraURL;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            if (RobotState.getInstance().getCurrentRobot().equals(Bot.Automatic)) {
                RobotState.getInstance().setCurrentRobot(Util.parseRobotNameToEnum(Util.readRobotName()));
            }
            switch (RobotState.getInstance().getCurrentRobot()) {
                case TestStand:
                    instance = new RobotHardwareTestStand();
                    break;
                case CompBot:
                    instance = new RobotHardwareCompBot();
                    break;
                case PracticeBot:
                    instance = new RobotHardwarePracticeBot();
                    break;
                case Automatic:
                default:
                    System.err.println("Robot should not be set to automatic... something went wrong");
                    break;
            }
            instance.initializeConstants();
            // Util.setPseudoInverseForwardKinematicsMatrix();
        }
        return instance;
    }

    public abstract void initializeConstants();
}
