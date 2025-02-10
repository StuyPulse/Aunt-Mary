/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.led;

import com.stuypulse.robot.subsystems.led.LEDController;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LedRainbow extends Command {
    protected final LEDController leds;

    public LedRainbow() {
        leds = LEDController.getInstance();
        addRequirements(leds);
    }

    @Override
    public void execute() {
        leds.applyPattern(LEDPattern.rainbow(255, 255));
    }
}
