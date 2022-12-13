package org.frogforce503.robot2022.commands.ClimbCommands;

import org.frogforce503.robot2022.subsystems.Climber;
import org.frogforce503.robot2022.subsystems.DriverFeedback;
import org.frogforce503.robot2022.subsystems.DriverFeedback.LEDStates;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimbThirdBarSequential extends SequentialCommandGroup {
    public ClimbThirdBarSequential() {
        addCommands(
                new RotateClimberCommand(true), // shift arm out for third bar
                new WaitCommand(0.5),
                new ClimberPositionCommand(29.5), // extend
                new PitchCommand(0.0, 10),
                new RotateClimberCommand(false), // shift arm into third bar
                new WaitCommand(0.3),
                new ClimberPositionCommand(17), // climb onto third bar

                new InstantCommand(() -> Climber.getInstance().setReadyToClimb(false)),
                new InstantCommand(() -> DriverFeedback.getInstance().setLEDState(LEDStates.DARK_GREEN))); // DONE!!!

    }

}
