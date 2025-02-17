package com.stuypulse.robot.commands.elevator;

import java.util.Optional;

import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorOverrideVoltage extends InstantCommand{
    private final Elevator elevator;
    private final double voltage;

    public ElevatorOverrideVoltage(double voltage) {
        this.elevator = Elevator.getInstance();
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        elevator.setVoltageOverride(Optional.of(voltage));
    }
}
