// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractUntilUnhooked extends CommandBase {
    double initialSetpoint;
    double increment;

    /**
     * Creates a new robotStateCommand which sets the current robot state to the
     * requested state.
     */
    public RetractUntilUnhooked() {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Intake.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialSetpoint = Climber.getInstance().getExtensionSetpoint();
        increment = -2;
    }

    @Override
    public void execute() {
        increment -= 4.0 / 50.0; // 6 inches / second
        System.out.println("Incrementing Climb by: " + (increment));

        Climber.getInstance().moveClimber(initialSetpoint + increment, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // retract by 1 more inch so we fully pull off bar
        Climber.getInstance().moveClimber(initialSetpoint + increment - 1.0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !Climber.getInstance().isFixedOnBar() && Climber.getInstance().climbEncodersAtSetpoint() ||
                this.increment <= -12.0; // if we've gone 4 inches and still have not made it just default to continue
                                         // on
    }
}
