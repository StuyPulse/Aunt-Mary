package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.math.util.Units;

public class ArmElevatorFeedForward extends Controller {

    private final Number kG;
    private final IStream elevatorAccelInGs;

    public ArmElevatorFeedForward(Number kG, IStream elevatorAccelInGs) {
        this.kG = kG;
        this.elevatorAccelInGs = elevatorAccelInGs;
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return kG.doubleValue() * Math.cos(Units.degreesToRadians(measurement)) * elevatorAccelInGs.get();
    }

}