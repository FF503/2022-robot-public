/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2022;

import edu.wpi.first.math.geometry.Translation2d;

public class RobotHardwareProgammingBot extends RobotHardware {
    /* All distance measurements are in inches, unless otherwise noted */

    @Override
    public void initializeConstants() {
        // Swerve Module JSON file names
        backLeftName = "AndyBackLeft";
        backRightName = "AndyBackRight";
        frontLeftName = "AndyFrontLeft";
        frontRightName = "AndyFrontRight";

        // Swerve Calculations Constants (measurements are in inches)
        kWheelbaseLength = 21.0;
        kWheelbaseWidth = 21.0;
        wheelDiameter = 4.0;
        // kTurnEncoderClicksperRevolution = 1024;
        // requestDriveReversed = -1;

        // requestPigeonFlipped = -1;

        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);

        kModulePositions = new Translation2d[] { kVehicleToBackRight, kVehicleToBackLeft, kVehicleToFrontLeft,
                kVehicleToFrontRight };

        // Pure Pursuit
        // kPathFollowingMaxAccel = 80;
        // kPathFollowingMaxVel = 130;

        // kMinLookAhead = 12.0; // inches
        // kMinLookAheadSpeed = 12.0; // inches per second
        // kMaxLookAhead = 48.0; // inches
        // kMaxLookAheadSpeed = kPathFollowingMaxVel; // inches per second

        // kPurePursuitV = 1 / kPathFollowingMaxVel;
        // kPurePursuitP = 0.0;

        // Gamespec
        panelSpinnerID = 27;
        panelSpinnerSolenoidID = 4;

        // Limelight
        limelightVerticalAngle = 20.25; // degrees
        limelightFloorClearance = 24; // inches
        magnifyThreshold = 0.0;

        // Targeting
        visionTargetingHeadingKp = 0.01;// 0.025;
        visionTargetingHeadingMinPower = 0.02;
        visionTargetingHeadingTxTolerance = 0.5;

        // Turret
        turretID = 7;
        kTurretMaxSpeed = 400;
        kTurretStartingAngle = 0;

    }
}
