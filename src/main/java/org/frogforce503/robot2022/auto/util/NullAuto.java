package org.frogforce503.robot2022.auto.util;

import org.frogforce503.lib.util.AutoRoutine;
import org.frogforce503.robot2022.commands.NewSwerveFollowPathCommand;

public class NullAuto implements AutoRoutine {

    @Override
    public void init() {
        System.out.println("WARNING: NULL AUTO RUNNING");
    }

    @Override
    public NewSwerveFollowPathCommand path() {
        return null;
    }

}
