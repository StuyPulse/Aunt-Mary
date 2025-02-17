package com.stuypulse.robot.commands.arm;

import java.util.Optional;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmOverrideVoltage extends InstantCommand{
    private final Arm arm;
    private final double voltage;

    public ArmOverrideVoltage(double voltage) {
        this.arm = Arm.getInstance();
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        arm.setVoltageOverride(Optional.of(voltage));
    }
}
