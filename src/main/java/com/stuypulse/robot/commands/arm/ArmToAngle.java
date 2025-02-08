/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.commands.led.LedSolidColor;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmToAngle extends InstantCommand{
    protected final Arm arm;
    protected final Rotation2d angle;

    public ArmToAngle(Rotation2d angle){
        arm = Arm.getInstance();
        this.angle = angle;
    }

    @Override
    public void initialize() {
        arm.setTargetAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return arm.atTargetAngle();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) new LedSolidColor(Color.kBlue).schedule();
    }
}
