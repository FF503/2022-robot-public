package org.frogforce503.robot2022.commands;

import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.SwerveControlState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoSnapCommand extends CommandBase {
    double targetAngle;
    Rotation2d targetRotation2d;
    double startTimestamp;

    boolean wasDrivingAboveTolerance = false;

    public AutoSnapCommand(double targetAngle) {
        this.targetAngle = targetAngle;
        this.targetRotation2d = Rotation2d.fromDegrees(targetAngle);
    }

    @Override
    public void initialize() {
        Swerve.getInstance().setStabilizationP(2.0);
        Swerve.getInstance().setStabilizationI(0.00005);
        startTimestamp = Timer.getFPGATimestamp();
        wasDrivingAboveTolerance = Swerve.getInstance().isDrivingAboveTolerance();
    }

    @Override
    public void execute() {
        System.out.println("TURNING ERROR IS: "
                + (Math.abs(this.targetAngle - Swerve.getInstance().getAngleDegrees())));
        Swerve.getInstance().snapToAngle(this.targetAngle);

        if (!wasDrivingAboveTolerance) {
            Swerve.getInstance().setPathFollowerSpeeds(new ChassisSpeeds(0, 0, Swerve.getInstance().getSnappingGain()));
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(this.targetAngle - Swerve.getInstance().getAngleDegrees()) <= 1)
                || Timer.getFPGATimestamp() - startTimestamp >= 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        if (edu.wpi.first.wpilibj.RobotState.isTeleop())
            Swerve.getInstance().setControlState(SwerveControlState.TELEOP);
        else
            Swerve.getInstance().setPathFollowerSpeeds(new ChassisSpeeds());

        Swerve.getInstance().setStabilizationP(0.0);
        Swerve.getInstance().setStabilizationI(0.0);
        Swerve.getInstance().disableSnapping();
    }
}
