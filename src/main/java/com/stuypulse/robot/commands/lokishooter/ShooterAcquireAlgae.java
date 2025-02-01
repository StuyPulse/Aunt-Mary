package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterAcquireAlgae extends InstantCommand {
    
    private final LokiShooter shooter;

    public ShooterAcquireAlgae(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.setSpeed(Settings.Shooter.ALGAE_ACQUIRE_SPEED.getAsDouble());
    }
}
