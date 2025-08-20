
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import com.stuypulse.stuylib.math.SLMath;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climb extends SubsystemBase {
    private static final Climb instance;

    static {
        instance = new ClimbImpl();
    }

    public static Climb getInstance() {
        return instance;
    }

    public enum ClimbState {
        CLOSED(Settings.Climb.CLOSED_ANGLE_DEG),
        OPEN(Settings.Climb.OPEN_ANGLE_DEG),
        CLIMBING(Settings.Climb.CLIMBED_ANGLE_DEG),
        SHIMMY(Settings.Climb.SHIMMY_ANGLE_DEG),
        IDLE(0); // Filler angle (wont be used)

        private double targetAngle;

        private ClimbState(double targetAngle) {
            this.targetAngle = targetAngle;
        }

        public double getTargetAngle() {
            return this.targetAngle;
        }
    }

    private ClimbState state;

    protected Climb() {
        this.state = ClimbState.CLOSED;
    }

    public ClimbState getState() {
        return this.state;
    }

    public void setState(ClimbState state) {
        this.state = state;
    }

    public abstract double getCurrentAngleDeg();

    @Override
    public void periodic() {
        SmartDashboard.putString("Climb/State", state.toString());
    }
}
