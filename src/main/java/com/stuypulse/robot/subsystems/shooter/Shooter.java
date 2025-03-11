/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;

    static {
        if (Robot.isReal()) {
            instance = new ShooterImpl();
        }
        else {
            instance = new ShooterSim();
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        ACQUIRE_CORAL(Settings.Shooter.CORAL_ACQUIRE_SPEED),
        ACQUIRE_ALGAE(Settings.Shooter.ALGAE_ACQUIRE_SPEED),
        SHOOT_CORAL_L1(Settings.Shooter.CORAL_SHOOT_SPEED_L1),
        SHOOT_CORAL_FORWARD(Settings.Shooter.CORAL_SHOOT_SPEED_FORWARD),
        SHOOT_CORAL_REVERSE(Settings.Shooter.CORAL_SHOOT_SPEED_REVERSE),
        SHOOT_ALGAE(Settings.Shooter.ALGAE_SHOOT_SPEED),
        HOLD_ALGAE(Settings.Shooter.ALGAE_HOLD_SPEED),
        STOP(0);

        private Number speed;

        private ShooterState(Number speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return this.speed.doubleValue();
        }
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

    public boolean shouldShootForward() {
        switch (Arm.getInstance().getState()) {
            case L2_FRONT, L2_BACK, L3_BACK, L4_BACK, L1:
                return true;
            default:
                return false;
        }
    }

    public boolean shouldShootBackwards() {
        switch (Arm.getInstance().getState()) {
            case L3_FRONT, L4_FRONT:
                return true;
            default:
                return false;
        }
    }

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.SHOOTER.get()) {
            RobotVisualizer.getInstance().updateShooter(getState().getSpeed(), hasCoral());
        }
        else {
            RobotVisualizer.getInstance().updateShooter(0, hasCoral());
        }
        SmartDashboard.putString("Shooter/State", getState().toString());
    }
}
