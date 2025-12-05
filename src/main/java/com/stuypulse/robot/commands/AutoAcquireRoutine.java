package com.stuypulse.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.vision.AlexServoToGamepiece;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoAcquireRoutine extends ParallelCommandGroup {
    private final LimelightVision vision;
    private Supplier<String> commandSelector;
    private String gamepiece;

    private final Command ALGAE = new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_ALGAE);
    private final Command CORAL = new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_CORAL);
    private final Command DEFAULT = new LEDApplyPattern(Settings.LED.AUTO_ACQUIRE_DEFAULT);;

    public AutoAcquireRoutine(Gamepad driver) {
        vision = LimelightVision.getInstance();
        gamepiece = vision.getCurrentGamepieceTarget();

        commandSelector = () -> {
            if (gamepiece.contains("algae")) {
                return "algae";
            } else if (gamepiece.contains("coral")) {
                return "coral";
            } else {
                System.out.println("DEFAULT CASE REACHED, HELP");
                return "none";
            }
        };

        addCommands(
            new AlexServoToGamepiece(driver)
                .alongWith(getLEDCommand(commandSelector))
        );
    }

    private Command getLEDCommand(Supplier<String> commandSelector) {
        Map<String, Command> commands = new HashMap<>();
        commands.put("algae", ALGAE);
        commands.put("coral", CORAL);
        commands.put("none", DEFAULT);
        
        return new SelectCommand<String>(commands, commandSelector);
    }
}
