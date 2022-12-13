package org.frogforce503.robot2022;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotHardwareCompBot extends RobotHardware {

        @Override
        public void initializeConstants() {
                this.backLeftName = "SDS6";
                this.backRightName = "SDS9";
                this.frontLeftName = "SDS4";
                this.frontRightName = "SDS2";

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

                limelightVerticalAngle = 29.0; // Y crosshair offset is -0.05 for fixed distances
                limelightFloorClearance = 39.0;
                magnifyThreshold = 0.0;

                limelightMagnifiedOffset = -13.6;
                // Gamespec

                // Shooter
                shooterID = 7;

                kShooterFalconKP = 0.8; // 0.8;// 0.07;// 0.04537;
                kShooterFalconKI = 0.001; // 0.0001;
                kShooterFalconKD = 0.0;
                kShooterFalconKS = 0.0;
                kShooterFalconKV = 0.04;// 0.05;// 0.06;
                kSHooterFalconkFF = kShooterFalconKV; // * (12 / 1023); // Talon Feedforward output conversion
                kShooterFalconKA = 0.0;
                kShooterFalconIZone = 0;

                kShooterRevsPerMotor = 1.0; // RATIO 2/1.57 // (28*2)/44 = 1.27
                kShooterIdle = 1500;// 1700;// 1909.0;
                kShooterBatterRPM = 2180.0;// 2050.0 // 1740.0;

                towerID = 2;
                towerBeamID = 0;

                kTowerRevsPerMotor = 1;// ADD Real value

                kTowerNeoKP = 0.000009;
                kTowerNeoKD = 0.0;
                kTowerNeoKV = 0.0001;
                kTowerNeoKS = 0.0; // ADD REAL VALUE
                kTowerFeedingVel = 5000;
                kTowerShootingVel = 9000; // increased in Tower shooting state// ADD REAL VALUE in RPM

                kMinHoodAngle = 40;
                kMaxHoodAngle = 168;
                kBatterHoodAngle = 44; // 46.0

                // Intake
                intakeID = 4;
                shiftForwardID = 0;
                shiftReverseID = 1;
                kIntakeRevsPerMotor = 1;// 16:36

                kIntakeNeoKP = 0.00008;
                kIntakeNeoKD = 4.0;// 0.015;
                kIntakeNeoKV = 0.000185;
                kIntakeNeoKS = 0.0; // ADD REAL VALUE
                kIntakeVelocity = 4200; // 4650 old max speed // ADD REAL VALUE in RPM
                kAutoIntakeVelocity = kIntakeVelocity;
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

                //
                isTurretInverted = false;
                turretHasGadgeteerLimits = true;
                turretID = 3;
                kTurretMaxSpeed = 400;
                kTurretStartingAngle = 90;
                kTurretKP = 3.2; // 4.0; // 3;
                kTurretKI = 0.0; // 0.001; // 0.006;
                kTurretKD = 170.0; // 80.0; // 5;// 20
                kTurretKFF = 1023.0 / 700.0;
                kTurretIzone = 40;
                kTurretCruiseV = 700.0;
                kTurretMA = 3000.0;
                kTurretMaxCCWLimit = 180;
                kTurretMaxCWLimit = -20;
                kTurretMaxCCWEncLimit = 5238;
                kTurretMaxCWEncLimit = 2864;

                kTurretSnapKP = 3.1;
                kTurretSnapKI = 0.0;
                kTurretSnapKD = 20;
                kTurretSnapKFF = 0;
                kTurretSnapIzone = 0;

                kTurretLockedKP = 8.0;
                kTurretLockedKI = 0.0;
                kTurretLockedKD = 0.0;
                kTurretLockedKFF = 0;

                kTurretEncoderBatterOffset = 92;
                kTurretEncoderHomeOffset = 1076;
                kTurretEncoderZeroOffset = 56;// 2740;Value Should match batter shot

                // kTurretEncoderFront = 2560;
                // kTurretEncoderBatter = 3576;
                // kTurretWideKP = 3;
                // kTurretWideKI = 0.00;
                // kTurretWideKD = 200;// 20
                // kTurretWideKFF = 0.0;

                // Hood
                leftLinearActuatorID = 4;// 1;
                rightLinearActuatorID = 5;// 2;
                kHoodSpeed = 26; // "Degrees" per second?
                // LED
                LED_ID = 0;

                // Climber

                hasClimber = true;
                climbLeftMotorID = 5;
                climbRightMotorID = 6;
                climberForwardID = 5;
                climberReverseID = 4;

                kConveyorRevsPerMotor = 1.0;
                kShooterRevsPerMotor = 1.0;

                leftClimbZeroID = 4;
                rightClimbZeroID = 5;
                leftFixedID = 7;
                rightFixedID = 6;

                // COLUMNS: DISTANCE, SHOOTER RPM, HOOD SETPOINT

                shotMapNewBalls = new double[][] {
                                { 61, 2190, 67 },
                                { 98, 2420, 88 },
                                { 127, 2560, 126 },
                                { 164, 2780, 146 },
                                { 193, 2940, 147 },
                                { 224, 3140, 147 },
                                { 277, 3560, 151 }
                };

                shotMapValues = shotMapNewBalls;
                limelightURL = "http://10.5.3.15:5800/";
                usbCameraURL = "http://10.5.3.2:1181/?action=stream";
        }

}
