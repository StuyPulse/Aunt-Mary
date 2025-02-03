package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootFront extends Command {
    private final LokiShooter shooter;

    public ShooterShootFront(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize(){
        if (shooter.hasCoral()){
            shooter.setSpeed(Settings.Shooter.CORAL_FRONT_SPEED.getAsDouble());
        }
    }

    @Override
    public boolean isFinished(){
        return !shooter.hasCoral();
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(0);
    }
}
