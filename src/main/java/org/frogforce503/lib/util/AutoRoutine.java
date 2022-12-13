package org.frogforce503.lib.util;

import org.frogforce503.robot2022.commands.NewSwerveFollowPathCommand;
import org.frogforce503.robot2022.commands.NoopCommand;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutoRoutine {
    default void init() {
    }

    default Command onInit() {
        return new NoopCommand();
    }

    default Command runCommand() {
        return new NoopCommand();
    }

    NewSwerveFollowPathCommand path();

    default void end() {
    }

    default void setup() {
        NewSwerveFollowPathCommand p = path();
        p.setup();
    }

    default Command onEnd() {
        return new NoopCommand();
    }

    default String getName() {
        return this.getClass().getSimpleName();
    }
}
