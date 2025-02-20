package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.numbers.IStream;

public class ArmDriveFeedForward extends Controller {

    private final Number kG;
    private final IStream forwardAccelerationInGs;

    public ArmDriveFeedForward(Number kG, IStream forwardAccelerationInGs) {
        this.kG = kG;
        this.forwardAccelerationInGs = forwardAccelerationInGs;
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return kG.doubleValue() * Math.sin(measurement) * forwardAccelerationInGs.get();
    }

}