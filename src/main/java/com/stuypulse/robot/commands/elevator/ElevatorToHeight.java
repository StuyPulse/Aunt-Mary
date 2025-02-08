/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.commands.led.LedSolidColor;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ElevatorToHeight extends InstantCommand {

    // public static Command untilDone(double height) {
    //     return new ElevatorToHeight(height)
    //             .andThen(new WaitUntilCommand(() -> Elevator.getInstance().atTargetHeight()));
    // }

    private final Elevator elevator;
    private final double targetHeight;

    public ElevatorToHeight(double targetHeight) {
        elevator = Elevator.getInstance();
        this.targetHeight = targetHeight;

        addRequirements(elevator);
    }

    public void initialize() {
        elevator.setTargetHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        return elevator.atTargetHeight();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) new LedSolidColor(LED.ABORT_COLOR).schedule();
    }
}