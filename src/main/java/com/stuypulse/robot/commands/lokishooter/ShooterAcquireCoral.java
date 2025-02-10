/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.commands.led.LedSolidColor;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAcquireCoral extends Command {

    private final LokiShooter shooter;

    public ShooterAcquireCoral() {
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(Settings.Shooter.CORAL_ACQUIRE_SPEED.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return shooter.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) new LedSolidColor(LED.ABORT_COLOR).schedule();
        shooter.setSpeed(0);
    }
}
