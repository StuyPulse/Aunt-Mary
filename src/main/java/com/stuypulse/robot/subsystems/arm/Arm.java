/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    public static final Arm instance;

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
        STOW,
        FEED,
        L2_FRONT,
        L3_FRONT,
        L4_FRONT,
        L2_BACK,
        L3_BACK,
        L4_BACK,
        ALGAE_L2,
        ALGAE_L3,
        BARGE,
        VERTICAL,
    }

    private ArmState state;

    protected Arm() {
        this.state = ArmState.STOW;
    }

    public ArmState getState() {
        return this.state;
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public abstract boolean atTargetAngle();

    @Override
    public void periodic() {
        ArmVisualizer.getInstance().update();

        SmartDashboard.putString("Arm/State", getState().toString());
        SmartDashboard.putBoolean("Arm/At Target Angle", atTargetAngle());
    }
}
