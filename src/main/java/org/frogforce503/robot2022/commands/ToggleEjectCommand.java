// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands;

import org.frogforce503.robot2022.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleEjectCommand extends CommandBase {

    /**
     * Creates a new IntakeStateCommand.
     */
    public ToggleEjectCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Intake.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Intake.getInstance().getIntakeState() != Intake.IntakeStates.EJECTING) {
            Intake.getInstance().setIntakeState(Intake.IntakeStates.EJECTING);
        } else {
            Intake.getInstance().setIntakeState(Intake.IntakeStates.OFF);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
