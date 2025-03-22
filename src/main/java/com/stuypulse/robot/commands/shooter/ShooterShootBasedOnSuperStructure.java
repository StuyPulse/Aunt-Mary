
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterShootBasedOnSuperStructure extends InstantCommand{
    private final Shooter shooter;

    public ShooterShootBasedOnSuperStructure() {
        this.shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    private ShooterState getTargetState() {
        return switch (SuperStructure.getInstance().getState()) {
            case L1 -> ShooterState.SHOOT_CORAL_L1;
            case L2_FRONT -> ShooterState.SHOOT_CORAL_L2_FRONT;
            case L2_BACK -> ShooterState.SHOOT_CORAL_L2_BACK;
            case L3_FRONT -> ShooterState.SHOOT_CORAL_L3_FRONT;
            case L3_BACK -> ShooterState.SHOOT_CORAL_L3_BACK;
            case L4_FRONT -> ShooterState.SHOOT_CORAL_L4_FRONT;
            case L4_BACK -> ShooterState.SHOOT_CORAL_L4_BACK;
            case PROCESSOR, BARGE_118 -> ShooterState.SHOOT_ALGAE;
            default -> ShooterState.SHOOT_CORAL_L1;
        };
    }

    @Override
    public void initialize() {
        shooter.setState(getTargetState());
    }
}
