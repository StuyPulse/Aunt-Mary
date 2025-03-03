/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
        L2_FRONT(Settings.Elevator.FRONT_L2_HEIGHT_METERS),
        L3_FRONT(Settings.Elevator.FRONT_L3_HEIGHT_METERS),
        L4_FRONT(Settings.Elevator.FRONT_L4_HEIGHT_METERS),
        L2_BACK(Settings.Elevator.BACK_L2_HEIGHT_METERS),
        L3_BACK(Settings.Elevator.BACK_L3_HEIGHT_METERS),
        L4_BACK(Settings.Elevator.BACK_L4_HEIGHT_METERS),
        ALGAE_L2(Settings.Elevator.ALGAE_L2_HEIGHT_METERS),
        ALGAE_L3(Settings.Elevator.ALGAE_L3_HEIGHT_METERS),
        BARGE(Settings.Elevator.BARGE_HEIGHT_METERS),
        PROCESSOR(Settings.Elevator.PROCESSOR_HEIGHT_METERS),
        BOTTOM(Constants.Elevator.MIN_HEIGHT_METERS),
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
        setOperatorOffset(0);
    }

    public ElevatorState getState() {
        return state;
    }

    public abstract double getCurrentHeight();
    public abstract boolean atTargetHeight();

    public abstract double getAccelGs();

    public abstract void setVoltageOverride(Optional<Double> voltage);
    public abstract void setOperatorOffset(double offset);
    public abstract double getOperatorOffset();

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setMotionProfileConstraints(double velLimitMetersPerSecond, double accelLimitMetersPerSecondSquared);

    @Override
    public void periodic() {
        RobotVisualizer.getInstance().updateElevatorHeight(getCurrentHeight(), atTargetHeight());

        SmartDashboard.putString("Elevator/State", state.toString());
        SmartDashboard.putNumber("Elevator/Target Height (m)", getState().getTargetHeight());
        SmartDashboard.putNumber("Elevator/Current Height (m)", getCurrentHeight());
        SmartDashboard.putBoolean("Elevator/At Target Height", atTargetHeight());
    }
}
