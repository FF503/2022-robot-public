package org.frogforce503.lib.util;

import org.frogforce503.robot2022.commands.NewSwerveFollowPathCommand;
import org.frogforce503.robot2022.commands.SwerveFollowPathCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// @formatter:off
public class AutoUtil {

    public static SequentialCommandGroup groupFor(SwerveFollowPathCommand path) {
        return new InstantCommand(path::saveStartingState)
            .andThen(path)
            .andThen(path::refreshStartingState);
    }

    public static SequentialCommandGroup groupFor(NewSwerveFollowPathCommand path) {
        return new InstantCommand(path::saveStartingState)
            .andThen(path)
            .andThen(path::refreshStartingState);
    }

    public static SequentialCommandGroup groupFor(Runnable init, SwerveFollowPathCommand path) {
        return new InstantCommand(path::saveStartingState)
            .andThen(init)
            .andThen(path)
            .andThen(path::refreshStartingState);
    }

    public static SequentialCommandGroup groupFor(Command init, SwerveFollowPathCommand path, Command end) {
        return new InstantCommand(path::saveStartingState)
            .andThen(init)
            .andThen(path)
            .andThen(path::refreshStartingState)
            .andThen(end);
    }

    public static SequentialCommandGroup groupFor(Command init, NewSwerveFollowPathCommand path, Command end) {
        return new InstantCommand(path::saveStartingState)
            .andThen(init)
            .andThen(path)
            .andThen(path::refreshStartingState)
            .andThen(end);
    }

    public static SequentialCommandGroup groupFor(Runnable init, SwerveFollowPathCommand path, Runnable end) {
        return groupFor(new InstantCommand(init), path, new InstantCommand(end));
    }

    public static SequentialCommandGroup groupFor(Runnable init, NewSwerveFollowPathCommand path, Runnable end) {
        return groupFor(new InstantCommand(init), path, new InstantCommand(end));
    }

    public static SequentialCommandGroup groupFor(Runnable init, SwerveFollowPathCommand path, Command end) {
        return groupFor(new InstantCommand(init), path, end);
    }

    public static SequentialCommandGroup groupFor(Runnable init, NewSwerveFollowPathCommand path, Command end) {
        return groupFor(new InstantCommand(init), path, end);
    }

    public static SequentialCommandGroup groupFor(Command init, SwerveFollowPathCommand path, Runnable end) {
        return groupFor(init, path, new InstantCommand(end));
    }

}
