package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.StateEngine.RobotStates;
import org.frogforce503.robot2022.commands.RobotStateCommand;
import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepForClimbSequential extends SequentialCommandGroup {
    /**
     * Creates a new ClimbFirstBarSequential Climb.
     */
    public PrepForClimbSequential() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new RobotStateCommand(RobotStates.CLIMBING),
                new RotateClimberCommand(false), // shifting in for first bar
                new RezeroClimberCommand(),
                new ClimberPositionCommand(28.5),
                // mark as ready to move to next stage
                new InstantCommand(() -> Climber.getInstance().setReadyToClimb(true)),
                new InstantCommand(() -> DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN)));
    }
}
