
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.led.LEDController;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class LEDApplyPattern extends Command {
    protected final LEDController leds;
    protected final Supplier<LEDPattern> pattern;

    public LEDApplyPattern(Supplier<LEDPattern> pattern) {
        leds = LEDController.getInstance();
        this.pattern = pattern;

        addRequirements(leds);
    }

    public LEDApplyPattern(LEDPattern pattern) {
        this(() -> pattern);
    }

    @Override
    public void execute() {
        leds.applyPattern(pattern.get());
    }
}
