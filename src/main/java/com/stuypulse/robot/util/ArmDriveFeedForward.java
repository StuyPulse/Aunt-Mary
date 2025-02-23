package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.math.util.Units;

public class ArmDriveFeedForward extends Controller {

    private final Number kG;
    private final IStream xAccelInGs;

    public ArmDriveFeedForward(Number kG, IStream xAccelInGs) {
        this.kG = kG;
        this.xAccelInGs = xAccelInGs;
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return -(kG.doubleValue() * Math.sin(Units.degreesToRadians(measurement)) * xAccelInGs.get());
    }

}