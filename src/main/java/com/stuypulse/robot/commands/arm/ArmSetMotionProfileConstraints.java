package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmSetMotionProfileConstraints extends InstantCommand {
    private Arm arm;
    private Rotation2d velLimit;
    private Rotation2d accelLimit;

    public ArmSetMotionProfileConstraints(Rotation2d velLimit, Rotation2d accelLimit) {
        arm = Arm.getInstance();

        this.velLimit = velLimit;
        this.accelLimit = accelLimit;
    }

    @Override
    public void initialize() {
        arm.setMotionProfileConstraints(velLimit, accelLimit);
    }
}
