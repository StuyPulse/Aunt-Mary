package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootAlgae extends Command {
    
    private final LokiShooter shooter;

    public ShooterShootAlgae(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {  
        shooter.setSpeed(Settings.Shooter.ALGAE_SHOOT_SPEED.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return !shooter.hasAlgae();
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(0);
    }
}
