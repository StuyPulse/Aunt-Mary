
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.funnel;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Funnel extends SubsystemBase {

    private static final Funnel instance;

    static {
        if (Robot.isReal()) {
            instance = new FunnelImpl();
        }
        else {
            instance = new FunnelSim();
        }
    }

    public static Funnel getInstance() {
        return instance;
    }

    public enum FunnelState {
        FORWARD(Settings.Funnel.FORWARD_SPEED),
        REVERSE(Settings.Funnel.REVERSE_SPEED),
        STOP(0);

        private Number speed;

        private FunnelState(Number speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return this.speed.doubleValue();
        }
    }

    private FunnelState state;

    protected Funnel() {
        this.state = FunnelState.STOP;
    }

    public FunnelState getState() {
        return state;
    }

    public void setState(FunnelState state) {
        this.state = state;
    }

    public abstract boolean shouldReverse();
    public abstract boolean hasCoral();

    @Override
    public void periodic() {
        SmartDashboard.putString("Funnel/State", getState().toString());
    }
}
