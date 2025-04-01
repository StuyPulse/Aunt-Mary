/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAcquireCoral extends Command {

    private final Shooter shooter;
    private final Funnel funnel;
    private final BStream isJammedInShooter;

    public ShooterAcquireCoral() {
        shooter = Shooter.getInstance();
        funnel = Funnel.getInstance();
        this.isJammedInShooter = BStream.create(() -> !shooter.hasCoral() && funnel.hasCoral())
            .filtered(new BDebounce.Rising(Settings.Shooter.SECONDS_BEFORE_REVERSING_TO_UNSTUCK_SHOOTER));

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (isJammedInShooter.get()) {
            shooter.setState(ShooterState.UNJAM_CORAL_BACKWARDS);
        } else {
            shooter.setState(ShooterState.ACQUIRE_CORAL);
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setState(ShooterState.STOP);
    }
}
