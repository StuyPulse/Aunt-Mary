package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.climb.ClimbClose;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class Reset extends ParallelCommandGroup{
    public Reset() {
        super(
            new SuperStructureFeed(),
            new ShooterStop(),
            new ClimbClose()
        );
    }
}
