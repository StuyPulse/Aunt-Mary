
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.led.LEDController;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDDisable extends InstantCommand{
    private final LEDController leds;

    public LEDDisable() {
        this.leds = LEDController.getInstance();
    }

    @Override
    public void initialize() {
        Settings.EnabledSubsystems.LEDS.set(false);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
