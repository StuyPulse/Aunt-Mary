/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import java.util.Optional;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Climb extends SubsystemBase {
    private static final Climb instance;

    static {
        instance = new ClimbImpl();
    }

    public static Climb getInstance() {
        return instance;
    }

    public enum ClimbState {
        CLOSED(Settings.Climb.CLOSED_ANGLE),
        OPEN(Settings.Climb.OPEN_ANGLE),
        CLIMBING(Settings.Climb.CLIMBED_ANGLE);

        private Rotation2d targetAngle;

        private ClimbState(Rotation2d targetAngle) {
            this.targetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(targetAngle.getDegrees(), Constants.Climb.MIN_ANGLE.getDegrees(), Constants.Climb.MAX_ANGLE.getDegrees()));
        }

        public Rotation2d getTargetAngle() {
            return this.targetAngle;
        }
    }

    private ClimbState state;

    protected Climb() {
        this.state = ClimbState.OPEN;
    }

    public ClimbState getState() {
        return this.state;
    }

    public void setState(ClimbState state) {
        this.state = state;
        setVoltageOverride(Optional.empty());
    }

    public abstract boolean atTargetAngle();

    public abstract void setVoltageOverride(Optional<Double> voltage);

    public abstract SysIdRoutine getSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putString("Climb/State", state.toString());
        SmartDashboard.putBoolean("Climb/At Target Angle", atTargetAngle());
    }
}
