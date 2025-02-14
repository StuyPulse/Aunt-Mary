package com.stuypulse.robot.commands.lokishooter;

import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterManual extends Command{
    private final LokiShooter shooter;
    private final double speed;

    public ShooterManual(double speed){
        shooter = LokiShooter.getInstance();
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setDutyCycle(speed);
    }
}
