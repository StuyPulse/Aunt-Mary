package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.CoralShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterAcquireAlgae extends InstantCommand {
    
    private final CoralShooter shooter;

    public ShooterAcquireAlgae(){
        shooter = CoralShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.setShooterRPM(Settings.Shooter.ALGAE_ACQUIRE_RPM.getAsDouble());
    }
}
