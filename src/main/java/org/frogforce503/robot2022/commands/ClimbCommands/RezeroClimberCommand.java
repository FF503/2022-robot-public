// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.OI;
import org.frogforce503.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RezeroClimberCommand extends CommandBase {

    boolean operatorOverride = false;

    /**
     * Creates a new robotStateCommand which sets the current robot state to the
     * requested state.
     */
    public RezeroClimberCommand() {
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
        if (!isFinished()) {
            System.out.println("trying to rezero");
            Climber.getInstance().rezeroClimber();
            operatorOverride = OI.getClimbOverride();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Climber.getInstance().zeroEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Climber.getInstance().isClimberZeroed() || operatorOverride;
    }
}
