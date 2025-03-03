/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Froggy extends SubsystemBase {

    public static final Froggy instance;

    static {
        if (Robot.isReal()) {
            instance = new FroggyImpl();
        }
        else {
            instance = new FroggySim();
        }
    }

    public static Froggy getInstance() {
        return instance;
    }

    public enum PivotState {
        STOW(Settings.Froggy.STOW_ANGLE),
        ALGAE_GROUND_PICKUP(Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE),
        CORAL_GROUND_PICKUP(Settings.Froggy.CORAL_GROUND_PICKUP_ANGLE),
        GOLF_TEE_ALGAE_PICKUP(Settings.Froggy.GOLF_TEE_ALGAE_PICKUP_ANGLE),
        L1_SCORE_ANGLE(Settings.Froggy.L1_SCORING_ANGLE),
        PROCESSOR_SCORE_ANGLE(Settings.Froggy.PROCESSOR_SCORE_ANGLE),
        CLIMB(Settings.Froggy.CLIMB_ANGLE);

        private Rotation2d targetAngle;

        private PivotState(Rotation2d targetAngle) {
            this.targetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(targetAngle.getDegrees(), Constants.Froggy.MINIMUM_ANGLE.getDegrees(), Constants.Froggy.MAXIMUM_ANGLE.getDegrees()));
        }

        public Rotation2d getTargetAngle() {
            return this.targetAngle;
        }
    }

    public enum RollerState {
        INTAKE_CORAL(Settings.Froggy.CORAL_INTAKE_SPEED),
        INTAKE_ALGAE(Settings.Froggy.ALGAE_INTAKE_SPEED),
        SHOOT_CORAL(Settings.Froggy.CORAL_OUTTAKE_SPEED),
        SHOOT_ALGAE(Settings.Froggy.ALGAE_OUTTAKE_SPEED),
        HOLD_ALGAE(Settings.Froggy.HOLD_ALGAE_SPEED),
        HOLD_CORAL(Settings.Froggy.HOLD_CORAL_SPEED),
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
        setPivotOperatorOffset(Rotation2d.kZero);
    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public abstract boolean isAtTargetAngle();
    public abstract boolean isStallingCoral();
    public abstract boolean isStallingAlgae();

    public abstract Rotation2d getCurrentAngle();

    public abstract void setPivotVoltageOverride(Optional<Double> voltage);
    public abstract void setPivotOperatorOffset(Rotation2d offset);
    public abstract Rotation2d getPivotOperatorOffset();

    public abstract SysIdRoutine getPivotSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putString("Froggy/Pivot State", getPivotState().toString());
        SmartDashboard.putString("Froggy/Roller State", getRollerState().toString());
    }
}
