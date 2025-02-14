/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        STOW(Constants.Elevator.MIN_HEIGHT_METERS),
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
        BOTTOM(Constants.Elevator.MIN_HEIGHT_METERS),
        CLEAR_FUNNEL(Constants.Elevator.FUNNEL_CLEAR_HEIGHT);

        private Number targetHeight;

        private ElevatorState(Number targetHeight) {
            this.targetHeight = targetHeight;
        }

        public double getTargetHeight() {
            return SLMath.clamp(targetHeight.doubleValue(), Constants.Elevator.MIN_HEIGHT_METERS, Constants.Elevator.MAX_HEIGHT_METERS);
        }
    }

    private ElevatorState state;
    private ElevatorVisualizer visualizer;

    public Elevator() {
        this.state = ElevatorState.STOW;
        this.visualizer = ElevatorVisualizer.getInstance();
    }

    public void setState(ElevatorState state) {
        this.state = state;
    }

    public ElevatorState getState() {
        return state;
    }

    public abstract double getCurrentHeight();
    public abstract boolean atTargetHeight();

    @Override
    public void periodic() {
        visualizer.update();

        SmartDashboard.putString("Elevator/State", state.toString());
        SmartDashboard.putNumber("Elevator/Target Height (m)", getState().getTargetHeight());
        SmartDashboard.putNumber("Elevator/Current Height (m)", getCurrentHeight());
    }
}
