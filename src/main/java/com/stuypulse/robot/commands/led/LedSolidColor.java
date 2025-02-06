package com.stuypulse.robot.commands.led;

import com.stuypulse.robot.subsystems.led.LEDController;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class LedSolidColor extends Command{
    LEDController controller;
    Color selectedColor;

    public LedSolidColor(Color color) {
        controller = LEDController.getInstance();
        selectedColor = color;
        
        addRequirements(controller);
    }

    @Override
    public void execute() {
        controller.applyPattern(LEDPattern.solid(selectedColor));
    }
}
