// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PitchCommand extends CommandBase {
    double pitch;
    double lastPitch;
    double changeInPitch;
    double maxPitch = 0;
    double minPitch = -10;
    double targetPitch = 0;

    /**
     * Creates a new robotStateCommand which sets the current robot state to the
     * requested state.
     */
    public PitchCommand(double target, double range) {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Intake.getInstance());
        this.targetPitch = target;
        this.maxPitch = target;
        this.minPitch = target - range;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastPitch = Swerve.getInstance().getPitch();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pitch = Swerve.getInstance().getPitch();
        changeInPitch = pitch - lastPitch;
        lastPitch = pitch;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { // OVERRIDE IN CASE SWITCHES BREAK DURING MATCH
        return pitch < maxPitch && pitch > minPitch && changeInPitch > 0;
    }
}
