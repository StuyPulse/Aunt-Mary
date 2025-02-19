package com.stuypulse.robot.commands.froggy.pivot;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.wpilibj2.command.Command;

public class FroggyPivotOverrideVoltage extends Command{
    private final Froggy froggy;
    private final Supplier<Double> voltage;

    public FroggyPivotOverrideVoltage(Supplier<Double> voltage) {
        this.froggy = Froggy.getInstance();
        this.voltage = voltage;
    }

    public FroggyPivotOverrideVoltage(double voltage) {
        this(() -> voltage);
    }

    @Override
    public void initialize() {
        froggy.setPivotVoltageOverride(Optional.of(voltage.get()));
    }

    @Override
    public void end(boolean interrupted) {
        froggy.setPivotVoltageOverride(Optional.of(0.0));
    }
}
