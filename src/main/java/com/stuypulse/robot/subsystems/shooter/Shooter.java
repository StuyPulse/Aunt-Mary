/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;

    static {
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        ACQUIRE_CORAL,
        ACQUIRE_ALGAE,
        SHOOT_CORAL_FORWARD,
        SHOOT_CORAL_REVERSE,
        SHOOT_ALGAE,
        STOP,
    }

    private ShooterState state;

    protected Shooter() {
        this.state = ShooterState.STOP;
    }

    public ShooterState getState() {
        return state;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public abstract boolean hasCoral();
    public abstract boolean isStalling();

    public boolean shouldShootForward() {
        switch (SuperStructure.getInstance().getTargetState()) {
            case L2_FRONT, L2_BACK, L3_BACK, L4_BACK:
                return true;
            default:
                return false;
        }
    }

    public boolean shouldShootBackwards() {
        switch (SuperStructure.getInstance().getTargetState()) {
            case L3_FRONT, L4_FRONT:
                return true;
            default:
                return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Shooter/State", getState().toString());
    }
}
