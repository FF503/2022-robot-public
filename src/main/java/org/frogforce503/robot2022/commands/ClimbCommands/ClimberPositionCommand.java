// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.Climber.ClimberStates;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberPositionCommand extends CommandBase {
    double inches;
    boolean slowSetpoint = false;
    boolean overridingClimb = false;

    /**
     * Creates a new ExtendClimber which sets the requested climb distance for the
     * climber
     */
    public ClimberPositionCommand(double inches, boolean slowSetpoint) {
        this.inches = inches;
        this.slowSetpoint = slowSetpoint;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Intake.getInstance());
    }

    public ClimberPositionCommand(double inches) {
        this.inches = inches;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Intake.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        overridingClimb = Climber.getInstance().getClimberState() == ClimberStates.TELEOP;
        if (!overridingClimb) {
            Climber.getInstance().moveClimber(inches, slowSetpoint);
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
        return Climber.getInstance().climbEncodersAtSetpoint()
                || overridingClimb;
    }
}
