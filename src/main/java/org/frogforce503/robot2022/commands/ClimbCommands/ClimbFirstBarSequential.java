package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;
import org.frogforce503.robot2022.subsystems.swerve.Swerve;
import org.frogforce503.robot2022.subsystems.swerve.Swerve.ModuleSnapPositions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbFirstBarSequential extends SequentialCommandGroup {
    /**
     * Creates a new ClimbFirstBarSequential Climb.
     */
    public ClimbFirstBarSequential() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
                // new ClimberPositionCommand(20), // CLIMB AND HOOK
                // new WaitCommand(1),
                new ClimberPositionCommand(0.5), // CLIMB AND HOOK
                new InstantCommand(() -> Swerve.getInstance().snapModulesTo(ModuleSnapPositions.STRAIGHT)),
                new WaitCommand(0.25),
                new ClimberPositionCommand(5, true), // LOWER ONTO FIXED HOOKS
                new IsFixedOnBarCommand(),
                new ClimberPositionCommand(7),
                // snap for exiting hangar on field dismount
                new InstantCommand(() -> Climber.getInstance().setReadyToClimb(true)),
                new InstantCommand(() -> DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN)));

        // DO NOTHING SINCE NOT IN CLIMBING STATE

        // ADD STEP OVER FUNCTIONS

    }
}
