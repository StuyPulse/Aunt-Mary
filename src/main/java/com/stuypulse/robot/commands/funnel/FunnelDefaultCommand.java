/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FunnelDefaultCommand extends Command {
    private final Funnel funnel;
    private final LokiShooter shooter;

    private boolean stopped = false;
    private boolean reversed = false;

    public FunnelDefaultCommand() {
        funnel = Funnel.getInstance();
        shooter = LokiShooter.getInstance();

        addRequirements(funnel);
    }

    @Override
    public void execute() {
        setState();

        runState();
    }

    private void setState() {
        stopped = shooter.hasCoral();

        if (funnel.isStalling() && !reversed) {
            reversed = true;

            new WaitCommand(1)
                    .andThen(
                            () -> {
                                reversed = false;
                            })
                    .schedule();
        }
    }

    private void runState() {
        if (stopped) {
            funnel.stop();
        } else if (reversed) {
            funnel.reverse();
        } else {
            funnel.forward();
        }
    }

    public boolean isUnjamming(){
        return reversed;
    }
}
