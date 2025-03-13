/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public abstract class ShooterSetState extends InstantCommand {

    private final Shooter shooter;
    private final ShooterState state;

    public ShooterSetState(ShooterState state) {
        shooter = Shooter.getInstance();
        this.state = state;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setState(state);
    }
}
