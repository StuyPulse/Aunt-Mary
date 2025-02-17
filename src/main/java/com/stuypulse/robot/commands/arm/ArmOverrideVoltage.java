package com.stuypulse.robot.commands.arm;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmOverrideVoltage extends Command{
    private final Arm arm;
    private final Supplier<Double> voltage;

    public ArmOverrideVoltage(Supplier<Double> voltage) {
        this.arm = Arm.getInstance();
        this.voltage = voltage;
    }

    public ArmOverrideVoltage(double voltage) {
        this(() -> voltage);
    }

    @Override
    public void initialize() {
        arm.setVoltageOverride(Optional.of(voltage.get()));
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltageOverride(Optional.of(0.0));
    }
}
