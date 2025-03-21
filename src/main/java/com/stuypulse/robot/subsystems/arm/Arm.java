/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
        FEED(Settings.Arm.FEED_ANGLE),
        L1(Settings.Arm.L1_ANGLE),
        L2_FRONT(Settings.Arm.L2_ANGLE_FRONT),
        L2_BACK(Settings.Arm.L2_ANGLE_BACK),
        L3_FRONT(Settings.Arm.L3_ANGLE_FRONT),
        L3_BACK(Settings.Arm.L3_ANGLE_BACK),
        L4_FRONT(Settings.Arm.L4_ANGLE_FRONT),
        L4_BACK(Settings.Arm.L4_ANGLE_BACK),
        ALGAE_L2(Settings.Arm.ALGAE_L2_ANGLE),
        ALGAE_L3(Settings.Arm.ALGAE_L3_ANGLE),
        PROCESSOR(Settings.Arm.PROCESSOR_ANGLE),
        CATAPULT_READY(Settings.Arm.CATAPULT_READY_ANGLE),
        CATAPULT_SHOOT(Settings.Arm.CATAPULT_FINAL_ANGLE),
        CLIMB(Settings.Arm.CLIMB_ANGLE),
        UNSTUCK_CORAL(Settings.Arm.UNSTUCK_CORAL_ANGLE);

        private Rotation2d targetAngle;

        private ArmState(Rotation2d targetAngle) {
            this.targetAngle = Rotation2d.fromDegrees(SLMath.clamp(targetAngle.getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
        }

        public Rotation2d getTargetAngle() {
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

    public abstract Rotation2d getCurrentAngle();
    public abstract boolean atTargetAngle();
    public abstract boolean canSkipClearance();

    public abstract void setVoltageOverride(Optional<Double> voltage);
    public abstract double getVoltageOverride();

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setMotionProfileConstraints(Rotation2d velLimit, Rotation2d accelLimit);

    @Override
    public void periodic() {
        RobotVisualizer.getInstance().updateArmAngle(getCurrentAngle(), atTargetAngle());
        
        SmartDashboard.putString("Arm/State", getState().toString());
        SmartDashboard.putBoolean("Arm/At Target Angle", atTargetAngle());

        SmartDashboard.putNumber("Arm/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Target Angle (deg)", getState().getTargetAngle().getDegrees());
    }
}
