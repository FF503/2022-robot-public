// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateClimberCommand extends CommandBase {
    boolean forward;

    /**
     * Creates a new ExtendClimber which sets the requested climb distance for the
     * climber
     */
    public RotateClimberCommand(boolean forward) {
        this.forward = forward;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Intake.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (forward) {
            Climber.getInstance().rotateUp();
        } else {
            Climber.getInstance().rotateDown();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true; // ISFINISHEd WHEN PNEUMATIC.get == REQUESTED POSITION
    }
}
