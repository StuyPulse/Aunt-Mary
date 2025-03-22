
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.shooter.ShooterShootL1;
import com.stuypulse.robot.commands.shooter.ShooterShootL2Back;
import com.stuypulse.robot.commands.shooter.ShooterShootL2Front;
import com.stuypulse.robot.commands.shooter.ShooterShootL3Back;
import com.stuypulse.robot.commands.shooter.ShooterShootL3Front;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Back;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        SHOOT_CORAL_L2_FRONT(Settings.Shooter.CORAL_SHOOT_SPEED_L2_FRONT),
        SHOOT_CORAL_L2_BACK(Settings.Shooter.CORAL_SHOOT_SPEED_L2_BACK),
        SHOOT_CORAL_L3_FRONT(Settings.Shooter.CORAL_SHOOT_SPEED_L3_FRONT),
        SHOOT_CORAL_L3_BACK(Settings.Shooter.CORAL_SHOOT_SPEED_L3_BACK),
        SHOOT_CORAL_L4_FRONT(Settings.Shooter.CORAL_SHOOT_SPEED_L4_FRONT),
        SHOOT_CORAL_L4_BACK(Settings.Shooter.CORAL_SHOOT_SPEED_L4_BACK),
        SHOOT_ALGAE(Settings.Shooter.ALGAE_SHOOT_SPEED),
        HOLD_ALGAE(Settings.Shooter.ALGAE_HOLD_SPEED),
        UNJAMB_CORAL_BACKWARDS(Settings.Shooter.UNJAMB_CORAL_BACKWARDS_SPEED),
        STOP(0);

        private double speed;

        private ShooterState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return this.speed;
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

    public boolean isShooting() {
        return switch (getState()) {
            case SHOOT_ALGAE, SHOOT_CORAL_L1, SHOOT_CORAL_L2_FRONT, SHOOT_CORAL_L2_BACK, SHOOT_CORAL_L3_FRONT, SHOOT_CORAL_L3_BACK, SHOOT_CORAL_L4_FRONT, SHOOT_CORAL_L4_BACK -> true;
            default -> false;
        };
    }

    public static Command getCorrespondingShootCommand(int level, boolean isScoringFrontSide) {
        switch (level) {
            case 1:
                return new ShooterShootL1();
            case 2:
                return isScoringFrontSide ? new ShooterShootL2Front() : new ShooterShootL2Back();  
            case 3:
                return isScoringFrontSide ? new ShooterShootL3Front() : new ShooterShootL3Back();       
            case 4:
                return isScoringFrontSide ? new ShooterShootL4Front() : new ShooterShootL4Back(); 
            default:
                return new ShooterShootL1();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Shooter/State", getState().toString());

        if (Settings.DEBUG_MODE) {
            if (Settings.EnabledSubsystems.SHOOTER.get()) {
                RobotVisualizer.getInstance().updateShooter(getState().getSpeed(), hasCoral());
            }
            else {
                RobotVisualizer.getInstance().updateShooter(0, hasCoral());
            }
        }
    }
}
