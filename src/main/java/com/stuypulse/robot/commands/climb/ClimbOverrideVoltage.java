package com.stuypulse.robot.commands.climb;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbOverrideVoltage extends Command{
    private final Climb climb;
    private final Supplier<Double> voltage;

    public ClimbOverrideVoltage(Supplier<Double> voltage) {
        this.climb = Climb.getInstance();
        this.voltage = voltage;
    }

    public ClimbOverrideVoltage(double voltage) {
        this(() -> voltage);
    }

    @Override
    public void initialize() {
        climb.setVoltageOverride(Optional.of(voltage.get()));
    }

    @Override
    public void end(boolean interrupted) {
        climb.setVoltageOverride(Optional.of(0.0));
    }
}
