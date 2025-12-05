package com.stuypulse.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.vision.AlexServoToGamepiece;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAcquireRoutine extends SequentialCommandGroup {
    private final Gamepad driver;
    private final LimelightVision vision;
    private Supplier<String> commandSelector;

    private final Command ALGAE = new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_ALGAE);
    private final Command CORAL = new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_CORAL);
    private final Command DEFAULT = new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR);

    public AutoAcquireRoutine(Gamepad driver) {
        this.driver = driver;
        vision = LimelightVision.getInstance();

        commandSelector = () -> {
            if (vision.getCurrentGamepieceTarget().contains("algae")) {
                return "algae";
            } else if (vision.getCurrentGamepieceTarget().contains("coral")) {
                return "coral";
            } else {
                return "none";
            }
        };

        addCommands(
            new AlexServoToGamepiece(driver),
            getSelectCommand(commandSelector)
        );
    }

    private Command getSelectCommand(Supplier<String> commandSelector) {
        Map<String, Command> commands = new HashMap<>();
        commands.put("algae", ALGAE);
        commands.put("coral", CORAL);
        commands.put("none", DEFAULT);

        return new SelectCommand(commands, commandSelector);
    }

}

// new ConditionalCommand(
//     new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_ALGAE), 
//     new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_ALGAE),
//     () -> vision.getCurrentGamepieceTarget().contains("algae")
// )
