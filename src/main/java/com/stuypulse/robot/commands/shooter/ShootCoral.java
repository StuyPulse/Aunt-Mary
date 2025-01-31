package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.CoralShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootCoral extends Command {
    private final CoralShooter shooter;

    public ShootCoral(){
        shooter = CoralShooter.getInstance();
        addRequirements(shooter);
    }
    
    @Override
    public void execute(){
        shooter.setShooterRPM(Settings.Shooter.MAX_SHOOTER_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRPM(0);
    }
}
