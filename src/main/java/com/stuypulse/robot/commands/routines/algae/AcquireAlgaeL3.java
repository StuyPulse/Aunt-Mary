/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.routines.algae;

import com.stuypulse.robot.commands.arm.algae.*;
import com.stuypulse.robot.commands.elevator.algae.*;
import com.stuypulse.robot.commands.lokishooter.ShooterAcquireAlgae;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AcquireAlgaeL3 extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;

    public AcquireAlgaeL3() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(new ElevatorToAlgaeL3(), new ArmToAlgaeL3(), new ShooterAcquireAlgae());
    }
}
