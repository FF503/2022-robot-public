package org.frogforce503.robot2022;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotHardwareTestStand extends RobotHardware {

    @Override
    public void initializeConstants() {
        this.backLeftName = "SDS4";

        // everything else in this file is copied from Programming bot as it was on
        // 1/11/2022
        this.backRightName = "ProgrammingBotBackRight";
        this.frontLeftName = "ProgrammingBotFrontLeft";
        this.frontRightName = "ProgrammingBotFrontRight";

        this.kWheelbaseLength = Units.inchesToMeters(24.75); // long side (left and right) (30 inch side)
        this.kWheelbaseWidth = Units.inchesToMeters(20.75); // short side (frotnt and back) (26 inch side)
        // full robot size is 32.625 x 36.625 inches (with bumpers)
        this.wheelDiameter = Units.inchesToMeters(4.0);

        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseWidth / 2, kWheelbaseLength / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseWidth / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseWidth / 2, -kWheelbaseLength / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseWidth / 2, kWheelbaseLength / 2);

        kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight, kVehicleToBackRight,
                kVehicleToBackLeft };

        visionTargetingRangeTolerance = 4;
        limelightTXTolerance = 0.5;

        limelightVerticalAngle = 20.0;
        limelightFloorClearance = 24.0;
        magnifyThreshold = 0.0;
    }

}
