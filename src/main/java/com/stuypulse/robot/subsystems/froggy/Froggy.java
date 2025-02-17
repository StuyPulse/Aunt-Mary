/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;

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
        INTAKE_CORAL(-Settings.Froggy.CORAL_INTAKE_SPEED),
        INTAKE_ALGAE(Settings.Froggy.ALGAE_INTAKE_SPEED),
        SHOOT_CORAL(Settings.Froggy.CORAL_OUTTAKE_SPEED),
        SHOOT_ALGAE(-Settings.Froggy.ALGAE_OUTTAKE_SPEED),
        HOLD_ALGAE(Settings.Froggy.HOLD_ALGAE_SPEED),
        STOP(0);

        private Number speed;

        private RollerState(Number speed) {
            this.speed = speed;
        }

        public Number getTargetSpeed() {
            return this.speed;
        }
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
        setPivotVoltageOverride(Optional.empty());
    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public abstract boolean isAtTargetAngle();
    public abstract boolean isStalling();

    public abstract void setPivotVoltageOverride(Optional<Double> voltage);

    @Override
    public void periodic() {
        SmartDashboard.putString("Froggy/Pivot State", getPivotState().toString());
        SmartDashboard.putString("Froggy/Roller State", getRollerState().toString());
    }
}
