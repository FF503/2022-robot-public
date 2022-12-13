// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands;

import org.frogforce503.robot2022.StateEngine;
import org.frogforce503.robot2022.StateEngine.RobotStates;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotStateCommand extends CommandBase {

    RobotStates inputState;

    /**
     * Creates a new robotStateCommand which sets the current robot state to the
     * requested state.
     */
    public RobotStateCommand(RobotStates inputState) {
        this.inputState = inputState;
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
        StateEngine.getInstance().setRobotState(inputState);
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
