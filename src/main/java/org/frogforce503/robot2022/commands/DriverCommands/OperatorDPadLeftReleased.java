package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.Hood;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OperatorDPadLeftReleased extends CommandBase {
    Constants.ShotPresets safePreset;

    @Override
    public void initialize() {
        safePreset = Constants.ShotPresets.LAUNCHPAD;

    }

    @Override
    public void execute() {
        StateEngine.getInstance().enablePresetOverride();
        Shooter.getInstance().setPresetShot(safePreset.getShooterRPM());
        Hood.getInstance().setManualHoodSetpoint(safePreset.getHoodSetpoint());
        if (StateEngine.getInstance().isPersistPreset()) {
            Turret.getInstance().setToPreset(safePreset.getTurretAngle());
        } else {
            Turret.getInstance().moveToTarget();
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
