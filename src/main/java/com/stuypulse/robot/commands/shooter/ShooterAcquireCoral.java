/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAcquireCoral extends Command {

    private final Shooter shooter;

    public ShooterAcquireCoral() {
        shooter = Shooter.getInstance();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setState(ShooterState.ACQUIRE_CORAL);
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
