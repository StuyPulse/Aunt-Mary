/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelDefaultCommand extends Command {
    
    private final Funnel funnel;
    private final Shooter shooter;
    private final BStream isJammedInShooter;
    

    public FunnelDefaultCommand() {
        this.funnel = Funnel.getInstance();
        this.shooter = Shooter.getInstance();
        this.isJammedInShooter = BStream.create(() -> !shooter.hasCoral() && funnel.hasCoral())
            .filtered(new BDebounce.Rising(Settings.Funnel.SECONDS_BEFORE_REVERSING_TO_UNSTUCK_SHOOTER));
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        if (shooter.hasCoral() 
            || shooter.getState() == ShooterState.HOLD_ALGAE 
            || Climb.getInstance().getState() != ClimbState.CLOSED
        ) {
            funnel.setState(FunnelState.STOP);
        }
        else if (funnel.shouldReverse() || isJammedInShooter.get()) {
            funnel.setState(FunnelState.REVERSE);
        }
        else {
            funnel.setState(FunnelState.FORWARD);
        }

    }
}
