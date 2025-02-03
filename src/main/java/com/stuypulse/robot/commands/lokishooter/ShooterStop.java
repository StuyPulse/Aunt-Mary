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