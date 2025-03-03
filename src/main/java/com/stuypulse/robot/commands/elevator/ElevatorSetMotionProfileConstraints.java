package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorSetMotionProfileConstraints extends InstantCommand {
    private Elevator elevator;
    private double velLimit;
    private double accelLimit;

    public ElevatorSetMotionProfileConstraints(double velLimit, double accelLimit) {
        elevator = Elevator.getInstance();

        this.velLimit = velLimit;
        this.accelLimit = accelLimit;
    }

    @Override
    public void initialize() {
        elevator.setMotionProfileConstraints(velLimit, accelLimit);
    }
}
