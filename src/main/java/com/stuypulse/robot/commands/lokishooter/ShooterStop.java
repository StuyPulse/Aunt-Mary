/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterStop extends InstantCommand {

    private final LokiShooter shooter;

    public ShooterStop() {
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(0);
    }
}
