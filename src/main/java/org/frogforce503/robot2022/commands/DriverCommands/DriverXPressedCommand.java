package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.ModuleSnapPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverXPressedCommand extends CommandBase {

    final double WALL_SHOT = 0; // FIXME

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (StateEngine.getInstance().getDriverControls()) {
            Shooter.getInstance()
                    .setShooterVelocityManual(Shooter.getInstance().getShooterSpeedForDistance(WALL_SHOT));
        } else {
            StateEngine.getInstance().setRobotState(RobotStates.REVERSE_TOWER);
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