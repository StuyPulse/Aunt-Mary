/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.routines.funnel_side;

import com.stuypulse.robot.commands.arm.funnel_side.*;
import com.stuypulse.robot.commands.elevator.funnel_side.*;
import com.stuypulse.robot.commands.lokishooter.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToL4Funnel extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;

    public MoveToL4Funnel() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(new ElevatorToL4Funnel(), new ArmToL4Funnel(), new ShooterShootFunnel());
    }
}
