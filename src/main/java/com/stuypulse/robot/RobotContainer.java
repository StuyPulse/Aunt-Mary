/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.commands.arm_elevator.*;
import com.stuypulse.robot.commands.arm_elevator.front_side.*;
import com.stuypulse.robot.commands.arm_elevator.funnel_side.*;
import com.stuypulse.robot.commands.arm_elevator.algae.*;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climb.ClimbDriveToClimb;
import com.stuypulse.robot.commands.climb.ClimbDriveToIntake;
import com.stuypulse.robot.commands.climb.ClimbDriveToStow;
import com.stuypulse.robot.commands.froggy.FroggyAlgaeGroundIntake;
import com.stuypulse.robot.commands.froggy.FroggyCoralGroundIntake;
import com.stuypulse.robot.commands.froggy.FroggyIntakeAlgae;
import com.stuypulse.robot.commands.froggy.FroggyOuttakeAlgae;
import com.stuypulse.robot.commands.froggy.FroggyProcessorScore;
import com.stuypulse.robot.commands.froggy.FroggyScoreL1;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.led.LedRainbow;
import com.stuypulse.robot.commands.led.LedSolidColor;
import com.stuypulse.robot.commands.lokishooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.lokishooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.lokishooter.ShooterShootFront;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.funnel.CoralFunnel;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem
    private final CoralFunnel funnel = CoralFunnel.getInstance();
    private final LokiShooter shooter = LokiShooter.getInstance();
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Froggy froggy = Froggy.getInstance();

    // public final LEDController ledController = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        new Trigger(() -> froggy.hasCoral() || shooter.hasCoral())
            .onTrue(new LedSolidColor(Color.kRed));
            
        new Trigger(((FunnelDefaultCommand) funnel.getDefaultCommand())::isUnjamming)
            .onTrue(new LedSolidColor(Color.kBlue));
        
        // Climb open, hooks go from stow to intake angle 
        new Trigger(() -> Math.abs(climb.getAngle().getDegrees() - Settings.Climb.OPEN_ANGLE) <= Settings.Climb.CLIMB_ANGLE_TOLERANCE)
            .onTrue(new LedRainbow());

        new Trigger(() -> Math.abs(climb.getAngle().getDegrees() - Settings.Climb.CLIMBED_ANGLE) <= Settings.Climb.CLIMB_ANGLE_TOLERANCE)
            .onTrue(new LedSolidColor(Color.kGreen));

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
    driver.getBottomButton()
        .whileTrue(new SequentialCommandGroup(
            new MoveToL2Front()
            .andThen(new ShooterShootFront()))); 
    
    // RIGHT BUTTON -> LVL 3 FRONT
    driver.getRightButton()
        .whileTrue(new SequentialCommandGroup(
            new MoveToL3Front()
            .andThen(new ShooterShootFront()))); 
    
    // TOP BUTTON -> LVL 4 FRONT
    driver.getTopButton()
        .whileTrue(new SequentialCommandGroup(
            new MoveToL4Front()
            .andThen(new ShooterShootFront()))); 

    // LEFT BUTTON -> BARGE SCORE
    driver.getLeftButton()
        .whileTrue(new SequentialCommandGroup(
            new MoveToBarge()
            .andThen(new ShooterShootAlgae())));
    
    /* PADDLES */
    
    // RIGHT TOP PADDLE -> CLIMB CLOSE
    driver.getLeftMenuButton()
        .onTrue(new ClimbDriveToStow());

    // RIGHT BOTTOM PADDLE -> CLIMB OPEN
    driver.getDPadRight()
        .onTrue(new ClimbDriveToIntake());

    // RIGHT MENU BUTTON -> CLIMB DRIVE
    driver.getRightMenuButton()
        .onTrue(new ClimbDriveToClimb());
    
    // LEFT TOP PADDLE -> L3 REEF ALGAE INTAKE
    driver.getDPadLeft()
        .whileTrue(new SequentialCommandGroup(
            new MoveToAlgaeL3()
            .andThen(new ShooterAcquireAlgae()))); // left top paddle -> get algae
        
    // LEFT BOTTOM PADDLE -> L2 REEF ALGAE INTAKE
    driver.getDPadDown()
        .whileTrue(new SequentialCommandGroup(
            new MoveToAlgaeL2()
            .andThen(new ShooterAcquireAlgae())));
    
    /* TRIGGERS */
    // RIGHT BUMPER -> L1 SCORE
    driver.getRightBumper()
        .whileTrue(new FroggyScoreL1());
    
    // RIGHT TRIGGER -> GROUND CORAL INTAKE
    driver.getRightTriggerButton()
        .whileTrue(new FroggyCoralGroundIntake());

    // LEFT BUMPER -> PROCESSOR SCORE
    driver.getLeftBumper()
        .whileTrue(new FroggyProcessorScore());
    
    // LEFT TRIGGER -> GROUND ALGAE INTAKE
    driver.getLeftTriggerButton()
        .whileTrue(new FroggyAlgaeGroundIntake());
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
