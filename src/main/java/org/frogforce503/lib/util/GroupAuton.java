/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.lib.util;

import org.frogforce503.lib.follower.Path;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public interface GroupAuton {
    SequentialCommandGroup getGroup();

    Path path();
}
