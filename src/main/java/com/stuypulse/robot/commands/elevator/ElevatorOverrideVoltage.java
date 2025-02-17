package com.stuypulse.robot.commands.elevator;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorOverrideVoltage extends Command{
    private final Elevator elevator;
    private final Supplier<Double> voltage;

    public ElevatorOverrideVoltage(Supplier<Double> voltage) {
        this.elevator = Elevator.getInstance();
        this.voltage = voltage;
    }

    public ElevatorOverrideVoltage(double voltage) {
        this(() -> voltage);
    }

    @Override
    public void initialize() {
        elevator.setVoltageOverride(Optional.of(voltage.get()));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltageOverride(Optional.of(0.0));
    }
}
