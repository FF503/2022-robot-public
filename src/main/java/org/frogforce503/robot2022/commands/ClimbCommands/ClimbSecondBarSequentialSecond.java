package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSecondBarSequentialSecond extends SequentialCommandGroup {
    /**
     * Creates a new ClimbSecondBarSequentialSecond Climb.
     */
    public ClimbSecondBarSequentialSecond() {
        addCommands(
                new ClimberPositionCommand(0.25), // PULL UP ONTO FIXED HOOKS
                new WaitCommand(0.25),
                new ClimberPositionCommand(5, true), // LOWER ONTO FIXED HOOKS
                new IsFixedOnBarCommand(),
                new ClimberPositionCommand(9), // LOWER ONTO FIXED HOOKS
                new InstantCommand(() -> Climber.getInstance().setReadyToClimb(true)),
                new InstantCommand(() -> DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN)));
        // CLIMB ON TO SECOND BAR

    }
}
