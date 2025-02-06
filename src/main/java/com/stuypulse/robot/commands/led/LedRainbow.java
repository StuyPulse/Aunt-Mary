package com.stuypulse.robot.commands.led;

import com.stuypulse.robot.subsystems.led.LEDController;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LedRainbow extends Command{
    LEDController controller;
    int value;

    public LedRainbow(int value) {
        controller = LEDController.getInstance();
        this.value = value;
        
        addRequirements(controller);
    }

    @Override
    public void execute() {
        controller.applyPattern(LEDPattern.rainbow(255, value));
    }
}
