package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.led.LEDController;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDEnable extends InstantCommand{
    private final LEDController leds;

    public LEDEnable() {
        this.leds = LEDController.getInstance();
    }

    @Override
    public void initialize() {
        Settings.EnabledSubsystems.LEDS.set(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
