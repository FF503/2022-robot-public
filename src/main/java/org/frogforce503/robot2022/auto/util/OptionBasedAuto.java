package org.frogforce503.robot2022.auto.util;

import java.util.HashMap;

import org.frogforce503.lib.util.AutoRoutine;
import org.frogforce503.robot2022.commands.NewSwerveFollowPathCommand;

public class OptionBasedAuto implements AutoRoutine {

    HashMap<String, AutoRoutine> autoRoutinesByLetter;

    public OptionBasedAuto(AutoRoutine a, AutoRoutine b, AutoRoutine c) {
        autoRoutinesByLetter = new HashMap<>() {
            {
                put("A", a);
                put("B", b);
                put("C", c);
            }
        };
    }

    public OptionBasedAuto(AutoRoutine a, AutoRoutine b) {
        this(a, b, null);
    }

    public OptionBasedAuto(AutoRoutine a) {
        this(a, null, null);
    }

    public AutoRoutine load(String option) {
        return autoRoutinesByLetter.get(option);
    }

    @Override
    public NewSwerveFollowPathCommand path() {
        return null;
    }
}
