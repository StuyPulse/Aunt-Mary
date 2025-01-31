package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.CoralShooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterShoot extends InstantCommand {
    private final CoralShooter shooter;

    public ShooterShoot(){
        shooter = CoralShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void initialize(){
        if (shooter.hasAlgae()){
            shooter.setShooterRPM(Settings.Shooter.ALGAE_TARGET_RPM.getAsDouble());
        } else if (shooter.hasCoral()){
            shooter.setShooterRPM(Settings.Shooter.CORAL_TARGET_RPM.getAsDouble());
        }
    }
}
