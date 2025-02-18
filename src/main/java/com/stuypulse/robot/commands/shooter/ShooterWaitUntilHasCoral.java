package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterWaitUntilHasCoral extends WaitUntilCommand{
    public ShooterWaitUntilHasCoral() {
        super(() -> Shooter.getInstance().hasCoral());
    }
}
