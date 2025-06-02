package com.stuypulse.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetConfigMode extends InstantCommand {
    public ShooterSetConfigMode(NeutralModeValue mode) {
        Shooter.getInstance().setMotorConfig(mode);
    }
}
