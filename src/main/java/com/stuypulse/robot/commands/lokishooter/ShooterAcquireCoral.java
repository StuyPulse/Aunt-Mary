package com.stuypulse.robot.commands.lokishooter;

import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

public class ShooterAcquireCoral extends Command {

    private final LokiShooter shooter;

    public ShooterAcquireCoral(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {  
        shooter.setSpeed(-Settings.Shooter.CORAL_ACQUIRE_SPEED.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return shooter.hasCoral();
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(0);
    }
}