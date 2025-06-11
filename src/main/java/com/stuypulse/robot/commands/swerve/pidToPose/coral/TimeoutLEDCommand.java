package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TimeoutLEDCommand extends Command {

        private final double timeoutSeconds;
        private final LEDApplyPattern initialCMD;
        private final LEDApplyPattern finalCMD;
        private final Timer timer;

        private boolean swapped = false;
        
        public TimeoutLEDCommand(LEDApplyPattern initialCMD, LEDApplyPattern finalCMD, double timeoutSeconds) {
                this.initialCMD = initialCMD;
                this.finalCMD = finalCMD;
                this.timeoutSeconds = timeoutSeconds;
                this.timer = new Timer();
        
        }
        
        @Override
        public void initialize() {
                timer.restart();
                initialCMD.schedule();

        }
        
        @Override
        public void execute() {
        if (!swapped && timer.hasElapsed(timeoutSeconds)) {
                initialCMD.cancel();
                finalCMD.schedule();
                swapped = true;
        }
        }
        
        @Override
        public boolean isFinished() {
        return false; 
        }
        
        @Override
        public void end(boolean interrupted) {
                if (!swapped) {
                        initialCMD.cancel();
                } else {
                        finalCMD.cancel();
                }
        }
        }
        