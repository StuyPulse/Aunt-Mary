package com.stuypulse.robot.subsystems.led;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    
    private static LEDController instance;
    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;
        
    private final LEDPattern defaultPattern = LEDPattern.kOff;
    
    static {
        instance = new LEDController(Ports.LED.LED_PORT, Constants.LED.LED_LENGTH);
    }

    public static LEDController getInstance() {
        return instance;
    }

    protected LEDController(int port, int length) {
        leds = new AddressableLED(port);
        ledsBuffer = new AddressableLEDBuffer(length);

        leds.setLength(length);
        leds.setData(ledsBuffer);
        leds.start();

        applyPattern(defaultPattern);

        SmartDashboard.putData(instance);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledsBuffer);
        leds.setData(ledsBuffer);
    }
}
