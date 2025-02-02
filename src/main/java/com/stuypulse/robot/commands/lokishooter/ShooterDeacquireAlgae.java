package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterDeacquireAlgae extends InstantCommand {
    
    private final LokiShooter shooter;

    public ShooterDeacquireAlgae(){
        shooter = LokiShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {  
        shooter.setSpeed(-Settings.Shooter.ALGAE_DEACQUIRE_SPEED.getAsDouble());
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
