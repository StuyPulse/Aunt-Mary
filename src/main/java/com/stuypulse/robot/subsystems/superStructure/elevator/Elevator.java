
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure.elevator;

import com.stuypulse.stuylib.math.SLMath;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public abstract class Elevator extends SubsystemBase {

    private static final Elevator instance;

    static {
        if (Robot.isReal()) {
            instance = new ElevatorImpl();
        } else {
            instance = new ElevatorSimu();
        }
    }

    public static Elevator getInstance() {
        return instance;
    }

    public enum ElevatorState {
        FEED(Settings.Elevator.FEED_HEIGHT_METERS),
        L1(Settings.Elevator.L1_HEIGHT_METERS),
        L2_FRONT(Settings.Elevator.FRONT_L2_HEIGHT_METERS),
        L3_FRONT(Settings.Elevator.FRONT_L3_HEIGHT_METERS),
        L4_FRONT(Settings.Elevator.FRONT_L4_HEIGHT_METERS),
        L2_BACK(Settings.Elevator.BACK_L2_HEIGHT_METERS),
        L3_BACK(Settings.Elevator.BACK_L3_HEIGHT_METERS),
        L4_BACK(Settings.Elevator.BACK_L4_HEIGHT_METERS),
        ALGAE_L2_FRONT(Settings.Elevator.ALGAE_L2_HEIGHT_METERS_FRONT),
        ALGAE_L3_FRONT(Settings.Elevator.ALGAE_L3_HEIGHT_METERS_FRONT),
        ALGAE_L2_BACK(Settings.Elevator.ALGAE_L2_HEIGHT_METERS_BACK),
        ALGAE_L3_BACK(Settings.Elevator.ALGAE_L3_HEIGHT_METERS_BACK),
        CATAPULT(Settings.Elevator.CATAPULT_HEIGHT_METERS),
        BARGE_118(Settings.Elevator.BARGE_118_HEIGHT_METERS),
        PROCESSOR(Settings.Elevator.PROCESSOR_HEIGHT_METERS),
        GOLF_TEE_ALGAE_PICKUP(Settings.Elevator.GOLF_TEE_ALGAE_PICKUP_HEIGHT),
        CLIMB(Settings.Elevator.CLIMB_HEIGHT_METERS),
        UNSTUCK_CORAL(Settings.Elevator.UNSTUCK_CORAL_HEIGHT_METERS);

        private Number targetHeight;

        private ElevatorState(Number targetHeight) {
            this.targetHeight = targetHeight;
        }

        public double getTargetHeight() {
            return SLMath.clamp(targetHeight.doubleValue(), Constants.Elevator.MIN_HEIGHT_METERS, Constants.Elevator.MAX_HEIGHT_METERS);
        }
    }

    private ElevatorState state;

    protected Elevator() {
        this.state = ElevatorState.FEED;
    }

    public void setState(ElevatorState state) {
        this.state = state;
        setVoltageOverride(Optional.empty());
    }

    public ElevatorState getState() {
        return state;
    }

    public static ElevatorState getState(int level, boolean isFrontFacingReef) {
        if (isFrontFacingReef) {
            if (level == 2) {
                return ElevatorState.L2_FRONT;
            } else if (level == 3) {
                return ElevatorState.L3_FRONT;
            } else if (level == 4) {
                return ElevatorState.L4_FRONT;
            }
        } else {
            if (level == 2) {
                return ElevatorState.L2_BACK;
            } else if (level == 3) {
                return ElevatorState.L3_BACK;
            } else if (level == 4) {
                return ElevatorState.L4_BACK;
            }
        }
        return ElevatorState.L1;
    }

    public abstract double getCurrentHeight();
    public abstract boolean atTargetHeight();
    public abstract boolean atCanSkipClearanceHeight();

    public abstract void setVoltageOverride(Optional<Double> voltage);

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setMotionProfileConstraints(double velLimitMetersPerSecond, double accelLimitMetersPerSecondSquared);

    @Override
    public void periodic() {
        SmartDashboard.putString("Elevator/State", state.toString());

        SmartDashboard.putNumber("Elevator/Target Height (m)", getState().getTargetHeight());
        SmartDashboard.putNumber("Elevator/Current Height (m)", getCurrentHeight());
        SmartDashboard.putBoolean("Elevator/At Target Height", atTargetHeight());

        if (Settings.DEBUG_MODE) {
            RobotVisualizer.getInstance().updateElevatorHeight(getCurrentHeight(), atTargetHeight());
        }
    }
}
