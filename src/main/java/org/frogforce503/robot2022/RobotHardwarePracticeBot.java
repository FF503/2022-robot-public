package org.frogforce503.robot2022;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotHardwarePracticeBot extends RobotHardware {

    @Override
    public void initializeConstants() {
        this.backLeftName = "SDS8";
        this.backRightName = "SDS6";
        this.frontLeftName = "SDS5";
        this.frontRightName = "SDS4";

        this.pigeonID = 9;

        this.kWheelbaseLength = Units.inchesToMeters(24.75); // long side (left and right) (30 inch side)
        this.kWheelbaseWidth = Units.inchesToMeters(20.75); // short side (frotnt and back) (26 inch side)
        // full robot size is 32.625 x 36.625 inches (with bumpers)
        this.wheelDiameter = Units.inchesToMeters(4);

        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseWidth / 2, kWheelbaseLength / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseWidth / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseWidth / 2, -kWheelbaseLength / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseWidth / 2, kWheelbaseLength / 2);

        kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight, kVehicleToBackRight,
                kVehicleToBackLeft };

        visionTargetingRangeTolerance = 4;
        limelightTXTolerance = 5.0;

        limelightVerticalAngle = 25.0;
        limelightFloorClearance = 45.0;
        magnifyThreshold = 0.0;

        limelightMagnifiedOffset = -10.7;

        // Swerve module calculation positions (weird cooridate system that we just kind
        // of have to deal with for CoR)

        // @formatter:off
        /*****
         *               ^
         *               +x
         *               |
         *               |
         *               |
         *               |
         *               |
         * <+y ------------------------ -y>
         *               |
         *               |
         *               |
         *               |
         *               |
         *               -x
         *               V
         */
        // @formatter:on

        double component = kWheelbaseLength / 2;

        // Gamespec

        // Shooter
        shooterID = 7;

        kShooterFalconKP = 0.8; // 1.0;// 0.07;// 0.04537;
        kShooterFalconKI = 0.001; // 0.004;
        kShooterFalconKD = 0.0;
        kShooterFalconKS = 0.0;
        kShooterFalconKV = 0.04;// 0.06;
        kSHooterFalconkFF = kShooterFalconKV; // * (12 / 1023); // Talon Feedforward output conversion
        kShooterFalconKA = 0.0;
        kShooterFalconIZone = 0;

        kShooterRevsPerMotor = 1.0; // RATIO 2/1.57 // (28*2)/44 = 1.27
        kShooterIdle = 1500;// 1909.0;
        kShooterBatterRPM = 2170.0;

        towerID = 2;
        towerBeamID = 0;

        kTowerRevsPerMotor = 1;// ADD Real value

        kTowerNeoKP = 0.00005;
        kTowerNeoKD = 1;// 0.015;
        kTowerNeoKV = 0.000097;
        kTowerNeoKS = 0.0; // ADD REAL VALUE
        kTowerShootingVel = 9000; // ADD REAL VALUE in RPM
        kTowerFeedingVel = 5000;

        kMinHoodAngle = 74;
        kMaxHoodAngle = 177;

        kBatterHoodAngle = 87.0;

        // Intake
        intakeID = 4;
        shiftForwardID = 0;
        shiftReverseID = 1;
        kIntakeRevsPerMotor = 1;// 16:36

        kIntakeNeoKP = 0.0004;
        kIntakeNeoKD = 4.0;// 0.015;
        kIntakeNeoKV = 0.00018;
        kIntakeNeoKS = 0.0; // ADD REAL VALUE
        kIntakeVelocity = 4500; // ADD REAL VALUE in RPM
        kAutoIntakeVelocity = 4500;
        kIntakeCurrentLimit = 60;

        // Conveyor
        hasPixyCam = true;
        conveyorID = 8;
        conveyorBeamID = 3;

        kConveyorRevsPerMotor = 1;// ADD Real value

        kConveyorNeoKP = 0.0001;
        kConveyorNeoKD = 0.8;// 0.015;
        kConveyorNeoKV = 0.000175;
        kConveyorNeoKS = 0.0; // ADD REAL VALUE
        kConveyorIntakingVelocity = 4500; // ADD REAL VALUE in RPM
        kConveyorVelocity = 5750; // ADD REAL VALUE in RPM

        // Turret
        isTurretInverted = false;
        turretHasGadgeteerLimits = true;
        turretID = 3;
        kTurretMaxSpeed = 400;
        kTurretStartingAngle = 90;
        kTurretKP = 3.0; // 4.0; // 3;
        kTurretKI = 0.0; // 0.001; // 0.006;
        kTurretKD = 110.0; // 80.0; // 5;// 20
        kTurretKFF = 1023.0 / 700.0;
        kTurretIzone = 40;
        kTurretCruiseV = 700.0;
        kTurretMA = 3000.0;
        kTurretMaxCCWLimit = 180;
        kTurretMaxCWLimit = -20;
        kTurretMaxCCWEncLimit = 3489;// 3933;
        kTurretMaxCWEncLimit = 1260;// 1680;

        kTurretSnapKP = 0.7;
        kTurretSnapKI = 0;
        kTurretSnapKD = 0;
        kTurretSnapKFF = 0;
        kTurretSnapIzone = 0;

        kTurretLockedKP = 8.0;
        kTurretLockedKI = 0.0;
        kTurretLockedKD = 0.0;
        kTurretLockedKFF = 0;
        kTurretLockedIzone = 0;

        kTurretEncoderBatterOffset = 92;
        kTurretEncoderHomeOffset = 1076;
        kTurretEncoderZeroOffset = 59;// 2740;Value Should match batter shot

        kTurretCwLimitID = 2;
        kTurretCCwLimitID = 1;

        // kTurretWideKP = 3;
        // kTurretWideKI = 0.00;
        // kTurretWideKD = 200;// 20
        // kTurretWideKFF = 0.0;

        // Hood
        leftLinearActuatorID = 1;
        rightLinearActuatorID = 2;
        kHoodSpeed = 26; // "Degrees" per second?
        // LED
        LED_ID = 0;

        // Climber
        hasClimber = false;

        climbLeftMotorID = 5;
        climbRightMotorID = 6;
        climberForwardID = 5;
        climberReverseID = 4;

        leftClimbZeroID = 5;
        rightClimbZeroID = 4;
        leftFixedID = 7;
        rightFixedID = 6;

        kConveyorRevsPerMotor = 1.0;

        shotMapValues = new double[][] {
                { 73, 2115, 117 },
                { 117, 2480, 135 },
                // { 155, 2770, 147 },
                { 157, 2740, 157 },
                // { 166, 2810, 150 },
                { 203, 3290, 177 }
                // { 203, 3210, 172 },

        };
        limelightURL = "http://10.5.2.15:5800/";
        usbCameraURL = "http://10.5.2.2:1181/?action=stream";
    }

}
