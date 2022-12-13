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
public class ClimbSecondBarSequential extends SequentialCommandGroup {
    /**
     * Creates a new ClimbSecondBarSequential Climb.
     */
    public ClimbSecondBarSequential() {
        addCommands(
                new RotateClimberCommand(true), // shifting out to reach to second bar
                new WaitCommand(0.45),
                new ClimberPositionCommand(29.5), // EXTEND
                new PitchCommand(0.0, 10.0),
                new RotateClimberCommand(false), // shifting arm into second bar
                new WaitCommand(0.25),
                new PitchCommand(-5.0, 15.0),
                new ClimberPositionCommand(12, true), // Pull off high bar
                new InstantCommand(() -> Climber.getInstance().setReadyToClimb(true)),
                new InstantCommand(() -> DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN)));
        // CLIMB ON TO SECOND BAR

    }
}
