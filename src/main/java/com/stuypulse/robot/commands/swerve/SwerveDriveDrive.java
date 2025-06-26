
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final Gamepad driver;

    private final VStream linearVelocity;
    private final IStream angularVelocity;

    public SwerveDriveDrive(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();

        linearVelocity = VStream.create(this::getDriverInputAsVelocity)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER),
                x -> x.mul(Climb.getInstance().getState() == ClimbState.CLOSED ? Drive.MAX_TELEOP_SPEED : Drive.MAX_TELEOP_SPEED_WHILE_CLIMBING),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC));

        angularVelocity = IStream.create(driver::getRightX)
            .filtered(
                x -> -x,
                x -> SLMath.deadband(x, Turn.DEADBAND),
                x -> SLMath.spow(x, Turn.POWER),
                x -> x * (Climb.getInstance().getState() == ClimbState.CLOSED ? Turn.MAX_TELEOP_TURN_SPEED : Turn.MAX_TELEOP_TURN_SPEED_WHILE_CLIMBING),
                new LowPassFilter(Turn.RC));

        this.driver = driver;

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(linearVelocity.get().x)
            .withVelocityY(linearVelocity.get().y)
            .withRotationalRate(angularVelocity.getAsDouble()));
    }
}