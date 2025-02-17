package com.stuypulse.robot.commands.froggy.pivot;

import java.util.Optional;

import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggyPivotOverrideVoltage extends InstantCommand{
    private final Froggy froggy;
    private final double voltage;

    public FroggyPivotOverrideVoltage(double voltage) {
        this.froggy = Froggy.getInstance();
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        froggy.setPivotVoltageOverride(Optional.of(voltage));
    }
}
