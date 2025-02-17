package com.stuypulse.robot.commands.climb;

import java.util.Optional;

import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimbOverrideVoltage extends InstantCommand{
    private final Climb climb;
    private final double voltage;

    public ClimbOverrideVoltage(double voltage) {
        this.climb = Climb.getInstance();
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        climb.setVoltageOverride(Optional.of(voltage));
    }
}
