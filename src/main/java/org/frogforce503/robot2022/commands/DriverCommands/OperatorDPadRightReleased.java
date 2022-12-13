package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.Hood;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OperatorDPadRightReleased extends CommandBase {
    Constants.ShotPresets tarmacPreset;

    @Override
    public void initialize() {
        tarmacPreset = Constants.ShotPresets.TARMAC_LINE;

    }

    @Override
    public void execute() {
        StateEngine.getInstance().enablePresetOverride();
        Shooter.getInstance().setPresetShot(tarmacPreset.getShooterRPM());
        Hood.getInstance().setManualHoodSetpoint(tarmacPreset.getHoodSetpoint());
        if (StateEngine.getInstance().isPersistPreset()) {
            Turret.getInstance().setToPreset(tarmacPreset.getTurretAngle());
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
