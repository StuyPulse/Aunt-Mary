/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.routines.front_side;

import com.stuypulse.robot.commands.arm.front_side.*;
import com.stuypulse.robot.commands.elevator.front_side.*;
import com.stuypulse.robot.commands.lokishooter.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreL2Front extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;

    public ScoreL2Front() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(new ArmToL2Front(), new ElevatorToL2Front(), new ShooterShootFront());
    }
}
