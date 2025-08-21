/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public abstract class Arm extends SubsystemBase {

    private static final Arm instance;

    static {
        if (Robot.isReal()) {
            instance = new ArmImpl();
        }
        else {
            instance = new ArmSim();
        }
    }

    public static Arm getInstance() {
        return instance;
    }

    public enum ArmState {
        FEED(Settings.Arm.FEED_ANGLE_DEG),
        L1_FRONT(Settings.Arm.L1_ANGLE_FRONT_DEG),
        L1_BACK(Settings.Arm.L1_ANGLE_BACK_DEG),
        L2_FRONT(Settings.Arm.L2_ANGLE_FRONT_DEG),
        L2_BACK(Settings.Arm.L2_ANGLE_BACK_DEG),
        L3_FRONT(Settings.Arm.L3_ANGLE_FRONT_DEG),
        L3_BACK(Settings.Arm.L3_ANGLE_BACK_DEG),
        L4_FRONT(Settings.Arm.L4_ANGLE_FRONT_DEG),
        L4_BACK(Settings.Arm.L4_ANGLE_BACK_DEG),
        AUTON_END(Settings.Arm.AUTON_END_DEG),
        ALGAE_L2_FRONT(Settings.Arm.ALGAE_L2_ANGLE_FRONT_DEG),
        ALGAE_L3_FRONT(Settings.Arm.ALGAE_L3_ANGLE_FRONT_DEG),
        ALGAE_L2_BACK(Settings.Arm.ALGAE_L2_ANGLE_BACK_DEG),
        ALGAE_L3_BACK(Settings.Arm.ALGAE_L3_ANGLE_BACK_DEG),
        GOLF_TEE_ALGAE_PICKUP(Settings.Arm.GOLF_TEE_ALGAE_PICKUP_ANGLE_DEG),
        GROUND_ALGAE_PICKUP(Settings.Arm.GROUND_ALGAE_PICKUP_ANGLE_DEG),
        PROCESSOR(Settings.Arm.PROCESSOR_ANGLE_DEG),
        CATAPULT_READY(Settings.Arm.CATAPULT_READY_ANGLE_DEG),
        CATAPULT_SHOOT(Settings.Arm.CATAPULT_FINAL_ANGLE_DEG),
        BARGE_118(Settings.Arm.BARGE_118_ANGLE_DEG),
        CLIMB(Settings.Arm.CLIMB_ANGLE_DEG),
        UNSTUCK_CORAL(Settings.Arm.UNSTUCK_CORAL_ANGLE_DEG),
        BARGE_SAFE_118(Settings.Arm.BARGE_SAFE_118_DEG);

        private double targetAngle;

        private ArmState(double targetAngle) {
            this.targetAngle = MathUtil.clamp(targetAngle, Settings.Arm.MIN_ANGLE_DEG, Settings.Arm.MAX_ANGLE_DEG);
        }

        public double getTargetAngle() {
            return this.targetAngle;
        }
    }

    private ArmState state;

    protected Arm() {
        this.state = ArmState.FEED;
    }

    public ArmState getState() {
        return this.state;
    }

    public void setState(ArmState state) {
        this.state = state;
        setVoltageOverride(Optional.empty());
    }

    public boolean isFunnelSide(ArmState state) {
        return state.getTargetAngle() > 90;
    }

    public abstract double getCurrentAngleDeg();
    public abstract boolean atTargetAngle();
    public abstract boolean atCanSkipClearanceAngle();

    public abstract void setVoltageOverride(Optional<Double> voltage);
    public abstract double getVoltageOverride();

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setMotionProfileConstraints(double velLimit, double accelLimit);

    @Override
    public void periodic() {        
        SmartDashboard.putString("Arm/State", getState().toString());
        SmartDashboard.putBoolean("Arm/At Target Angle", atTargetAngle());

        SmartDashboard.putNumber("Arm/Current Angle (deg)", getCurrentAngleDeg());
        SmartDashboard.putNumber("Arm/Target Angle (deg)", getState().getTargetAngle());

        if (Settings.DEBUG_MODE) {
            RobotVisualizer.getInstance().updateArmAngle(getCurrentAngleDeg(), atTargetAngle());
        }
    }
}
