package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.ModuleSnapPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverACommand extends CommandBase {

    final double SAFE_ZONE_DISTANCE = 0; // FIXME

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (StateEngine.getInstance().getDriverControls()) {
            Shooter.getInstance()
                    .setShooterVelocityManual(Shooter.getInstance().getShooterSpeedForDistance(SAFE_ZONE_DISTANCE));
        } else {
            Swerve.getInstance().snapModulesTo(ModuleSnapPositions.STRAIGHT);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
