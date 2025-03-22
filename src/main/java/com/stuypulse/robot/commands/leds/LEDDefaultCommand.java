
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultCommand extends Command{
    private final LEDController leds;
    private final SuperStructure superStructure;
    private final Shooter shooter;
    private final Froggy froggy;
    private final Funnel funnel;
    private final Climb climb;

    public LEDDefaultCommand() {
        this.leds = LEDController.getInstance();
        this.superStructure = SuperStructure.getInstance();
        this.shooter = Shooter.getInstance();
        this.froggy = Froggy.getInstance();
        this.funnel = Funnel.getInstance();
        this.climb = Climb.getInstance();
        addRequirements(leds);
    }

    private boolean isScoring() {
        return shooter.isShooting()
            || shooter.getState() == ShooterState.SHOOT_ALGAE
            || froggy.getRollerState() == RollerState.SHOOT_CORAL 
            || froggy.getRollerState() == RollerState.SHOOT_ALGAE;
    }

    @Override
    public void execute() {
        if (isScoring()) {
            leds.applyPattern(Settings.LED.SCORE_COLOR);
        }
        else if (climb.getState() == ClimbState.OPEN) {
            leds.applyPattern(Settings.LED.CLIMB_OPEN_COLOR);
        }
        else if (climb.getState() == ClimbState.SHIMMY) {
            leds.applyPattern(Settings.LED.SHIMMY_COLOR);
        }
        else if (climb.getState() == ClimbState.CLIMBING) {
            leds.applyPattern(Settings.LED.CLIMBING_COLOR);
        }
        else if (funnel.getState() == FunnelState.REVERSE) {
            leds.applyPattern(Settings.LED.FUNNEL_UNJAM_COLOR);
        }
        else if (froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE || superStructure.getState() == SuperStructureState.PROCESSOR) {
            leds.applyPattern(Settings.LED.PROCESSOR_SCORE_ANGLE);
        }
        else if (shooter.hasCoral() || funnel.hasCoral()) {
            leds.applyPattern(Settings.LED.HAS_CORAL_COLOR);
        }
        else {
            leds.applyPattern(LEDPattern.kOff);
        }
    } 

}
