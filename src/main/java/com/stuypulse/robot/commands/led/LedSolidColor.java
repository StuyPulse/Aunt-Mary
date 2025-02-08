// package com.stuypulse.robot.commands.led;

// import com.stuypulse.robot.subsystems.led.LEDController;

// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.Command;

public class LedSolidColor extends Command{
    protected final LEDController leds;
    protected final Color selectedColor;

    public LedSolidColor(Color color) {
        leds = LEDController.getInstance();
        selectedColor = color;
        
        addRequirements(leds);
    }

    @Override
    public void execute() {
        leds.applyPattern(LEDPattern.solid(selectedColor));
    }
}
