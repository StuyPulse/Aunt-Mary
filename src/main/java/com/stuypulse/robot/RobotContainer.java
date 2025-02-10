/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climb.*;
import com.stuypulse.robot.commands.froggy.*;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.led.LedRainbow;
import com.stuypulse.robot.commands.led.LedSolidColor;
import com.stuypulse.robot.commands.routines.*;
import com.stuypulse.robot.commands.routines.algae.*;
import com.stuypulse.robot.commands.routines.front_side.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem
    private final Funnel funnel = Funnel.getInstance();
    private final LokiShooter shooter = LokiShooter.getInstance();
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Froggy froggy = Froggy.getInstance();
    private final LEDController ledController = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        new Trigger(() -> froggy.hasCoral() || shooter.hasCoral())
                .onTrue(new LedSolidColor(LED.HAS_CORAL_COLOR));

        new Trigger(((FunnelDefaultCommand) funnel.getDefaultCommand())::isUnjamming)
                .onTrue(new LedSolidColor(LED.UNJAM_COLOR));

        new Trigger(
                        () ->
                                (Math.abs(
                                                climb.getAngle().getDegrees()
                                                        - Settings.Climb.OPEN_ANGLE.getDegrees())
                                        <= Settings.Climb.ANGLE_TOLERANCE.getDegrees()))
                .onTrue(new LedRainbow());

        new Trigger(
                        () ->
                                (Math.abs(
                                                climb.getAngle().getDegrees()
                                                        - Settings.Climb.CLIMBED_ANGLE.getDegrees())
                                        <= Settings.Climb.ANGLE_TOLERANCE.getDegrees()))
                .onTrue(new LedSolidColor(LED.CLIMB_COLOR));
    }

    /****************/
    /*** DEFAULT ***/
    /****************/

    private void configureDefaultCommands() {
        funnel.setDefaultCommand(new FunnelDefaultCommand());
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureButtonBindings() {

        /* RIGHT SIDE BUTTONS */
        // ADD ALIGNMENT TO ALL SCORING ROUTINES

        // BOTTOM BUTTON -> LVL 2 FRONT
        driver.getBottomButton().whileTrue(new ScoreL2Front()).onFalse(new MoveToFeed());

        // RIGHT BUTTON -> LVL 3 FRONT
        driver.getRightButton().whileTrue(new ScoreL3Front()).onFalse(new MoveToFeed());

        // TOP BUTTON -> LVL 4 FRONT
        driver.getTopButton().whileTrue(new ScoreL4Front()).onFalse(new MoveToFeedReverse());

        // LEFT BUTTON -> BARGE SCORE
        driver.getLeftButton().whileTrue(new ScoreBarge()).onFalse(new MoveToFeed());

        // TOP RIGHT PADDLE -> CLIMB CLOSE
        driver.getLeftMenuButton().onTrue(new ClimbStow());

        // BOTTOM RIGHT PADDLE -> CLIMB OPEN
        driver.getDPadRight().onTrue(new ClimbOpen());

        // RIGHT MENU BUTTON -> CLIMB DRIVE
        driver.getRightMenuButton().onTrue(new ClimbClimb());

        // TOP LEFT PADDLE -> L3 REEF ALGAE INTAKE
        driver.getDPadLeft().whileTrue(new AcquireAlgaeL3()).onFalse(new MoveToStow());

        // BOTTOM LEFT PADDLE -> L2 REEF ALGAE INTAKE
        driver.getDPadDown().whileTrue(new AcquireAlgaeL2()).onFalse(new MoveToStow());

        // RIGHT BUMPER -> L1 SCORE
        driver.getRightBumper().whileTrue(new FroggyScoreL1());

        // RIGHT TRIGGER -> GROUND CORAL INTAKE
        driver.getRightTriggerButton().whileTrue(new FroggyCoralGroundIntake());

        // LEFT BUMPER -> PROCESSOR SCORE
        driver.getLeftBumper().whileTrue(new FroggyProcessorScore());

        // LEFT TRIGGER -> GROUND ALGAE INTAKE
        driver.getLeftTriggerButton().whileTrue(new FroggyAlgaeGroundIntake());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
