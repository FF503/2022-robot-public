package org.frogforce503.lib.follower;

import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * FF Wrapper for a HolonomicDriveController with a SEPARATE heading controller
 * (because ProfilePIDController is problematic with our drivetrain)
 */
public class SwervePathFollower {

    private HolonomicDriveController driveController;

    public static final double FOLLOWER_LOOP_TIME_INTERVAL = 20 / 1000; // running on 20 ms loop
    public static final double FOLLOWER_END_DISTANCE_TOLERANCE = Units.inchesToMeters(2.0); // 10 cm

    public SwervePathFollower(PIDController xController, PIDController yController,
            ProfiledPIDController thetaController) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveController = new HolonomicDriveController(
                xController,
                yController,
                thetaController);
    }

    public ChassisSpeeds toPose(Pose2d targetPose) {
        return this.toPose(targetPose, 0);
    }

    public ChassisSpeeds toPose(Pose2d targetPose, double velocity) {
        ChassisSpeeds controllerOutput = driveController.calculate(Swerve.getInstance().getPoseMeters(), targetPose,
                velocity,
                targetPose.getRotation());

        SmartDashboard.putNumber("HEADING TARGET", targetPose.getRotation().getDegrees());

        boolean outputInvalid = Double.isNaN(controllerOutput.vxMetersPerSecond)
                || Double.isNaN(controllerOutput.vyMetersPerSecond)
                || Double.isNaN(controllerOutput.omegaRadiansPerSecond);
        if (outputInvalid)
            return new ChassisSpeeds(0, 0, 0);

        // thetaController.setSetpoint(targetPose.getRotation().getDegrees());

        // double thetaGain =
        // thetaController.calculateOutput(Swerve.getInstance().getAngleDegrees(),
        // true);
        // outputInvalid = (Math
        // .abs(targetPose.getRotation().minus(Swerve.getInstance().getAngleRotation2d()).getDegrees())
        // < 2)
        // || Double.isNaN(thetaGain);

        // controllerOutput.omegaRadiansPerSecond = outputInvalid ? 0
        // : thetaGain;

        return controllerOutput;
    }
}