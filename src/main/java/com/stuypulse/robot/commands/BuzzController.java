package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.Command;

public class BuzzController extends Command {
    private final Gamepad driver;
    private final StopWatch timer;

    public BuzzController(Gamepad driver) {
        this.driver = driver;
        timer = new StopWatch();
    }

    @Override
    public void initialize() {
        driver.setRumble(Settings.Driver.BUZZ_INTENSITY);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.getTime() >= Settings.Driver.BUZZ_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(0);
    }
}