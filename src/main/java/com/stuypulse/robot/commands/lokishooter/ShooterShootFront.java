package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterShootFront extends InstantCommand {
    private final LokiShooter shooter;

    public ShooterShootFront(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize(){
        if (shooter.hasAlgae()){
            shooter.setSpeed(Settings.Shooter.ALGAE_FRONT_SPEED.getAsDouble());
        } else if (shooter.hasCoral()){
            shooter.setSpeed(Settings.Shooter.CORAL_FRONT_SPEED.getAsDouble());
        }
    }
}
