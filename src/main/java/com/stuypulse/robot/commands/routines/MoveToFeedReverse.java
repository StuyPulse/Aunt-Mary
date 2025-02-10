/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.routines;

import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.elevator.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToFeedReverse extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;

    public MoveToFeedReverse() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        addRequirements(arm, elevator);

        addCommands(new ArmToFeedReverse(), new ElevatorToFeed());
    }
}
