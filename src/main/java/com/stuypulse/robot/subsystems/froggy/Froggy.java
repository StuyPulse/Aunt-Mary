/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Froggy extends SubsystemBase {

    public static final Froggy instance;

    static {
        instance = new FroggyImpl();
    }

    public static Froggy getInstance() {
        return instance;
    }

    public enum PivotState {
        STOW,
        ALGAE_GROUND_PICKUP,
        CORAL_GROUND_PICKUP,
        GOLF_TEE_ALGAE_PICKUP,
        L1_SCORE_ANGLE,
        PROCESSOR_SCORE_ANGLE
    }

    public enum RollerState {
        INTAKE_CORAL,
        INTAKE_ALGAE,
        SHOOT_CORAL,
        SHOOT_ALGAE,
        HOLD_ALGAE,
        STOP
    }

    private PivotState pivotState;
    private RollerState rollerState;

    protected Froggy() {
        this.pivotState = PivotState.STOW;
        this.rollerState = RollerState.STOP;
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public void setPivotState(PivotState state) {
        this.pivotState = state;
    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public abstract boolean isAtTargetAngle();
    public abstract boolean isStalling();

    @Override
    public void periodic() {
        SmartDashboard.putString("Froggy/Pivot State", getPivotState().toString());
        SmartDashboard.putString("Froggy/Roller State", getRollerState().toString());
    }
}
