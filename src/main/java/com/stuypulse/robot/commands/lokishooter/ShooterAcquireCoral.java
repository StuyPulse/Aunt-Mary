package com.stuypulse.robot.commands.lokishooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

public class ShooterAcquireCoral extends InstantCommand {

    private final LokiShooter shooter;

    public ShooterAcquireCoral(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {  
        shooter.setSpeed(-Settings.Shooter.CORAL_ACQUIRE_SPEED.getAsDouble());
    }
}