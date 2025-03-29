
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.math.util.Units;

public class ArmDriveFeedForward extends Controller {

    private final Number kG;
    private final IStream forwardAccelInGs;

    public ArmDriveFeedForward(Number kG, IStream forwardAccelInGs) {
        this.kG = kG;
        this.forwardAccelInGs = forwardAccelInGs;
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return -(kG.doubleValue() * Math.sin(Units.degreesToRadians(measurement)) * forwardAccelInGs.get());
    }

}