/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import java.util.Optional;

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
        STOW,
        OPEN,
        CLIMBING,
    }

    private ClimbState state;

    protected Climb() {
        this.state = ClimbState.STOW;
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

    public abstract Command getSysIdQuasistatic(SysIdRoutine.Direction direction);
    public abstract Command getSysIdDynamic(SysIdRoutine.Direction direction);

    @Override
    public void periodic() {
        SmartDashboard.putString("Climb/State", state.toString());
        SmartDashboard.putBoolean("Climb/At Target Angle", atTargetAngle());
    }
}
