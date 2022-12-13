package org.frogforce503.robot2022.commands.DriverCommands;

import org.frogforce503.robot2022.Constants;
import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.subsystems.Hood;
import org.frogforce503.robot2022.subsystems.Shooter;
import org.frogforce503.robot2022.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OperatorDPadUpReleased extends CommandBase {
    Constants.ShotPresets batterPreset;

    @Override
    public void initialize() {
        batterPreset = Constants.ShotPresets.BATTER;
    }

    @Override
    public void execute() {
        StateEngine.getInstance().enablePresetOverride();
        Shooter.getInstance().setPresetShot(batterPreset.getShooterRPM());
        Hood.getInstance().setManualHoodSetpoint(batterPreset.getHoodSetpoint());
        Turret.getInstance().setToPreset(batterPreset.getTurretAngle());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
