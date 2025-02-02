package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterShootBack extends Command {
    private final LokiShooter shooter;

    public ShooterShootBack(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize(){
        if (shooter.hasAlgae()){
            shooter.setSpeed(-Settings.Shooter.ALGAE_BACK_SPEED.getAsDouble());
        } else if (shooter.hasCoral()){
            shooter.setSpeed(-Settings.Shooter.CORAL_BACK_SPEED.getAsDouble());
        }
    }

    @Override
    public boolean isFinished(){
        return !shooter.hasAlgae() && !shooter.hasCoral();
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(0);
    }
}
