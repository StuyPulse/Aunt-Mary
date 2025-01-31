package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.CoralShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterStop extends InstantCommand {

    private final CoralShooter shooter;

    public ShooterStop() {
        shooter = CoralShooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterRPM(0);
    }
}