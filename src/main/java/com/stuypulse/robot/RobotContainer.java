/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.commands.arm.ArmMoveToBarge;
import com.stuypulse.robot.commands.arm.ArmMoveToL2Front;
import com.stuypulse.robot.commands.arm.ArmMoveToL3Front;
import com.stuypulse.robot.commands.arm.ArmMoveToL4Front;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climb.ClimbDriveToClimb;
import com.stuypulse.robot.commands.climb.ClimbDriveToIntake;
import com.stuypulse.robot.commands.climb.ClimbDriveToStow;
import com.stuypulse.robot.commands.elevator.ElevatorToBarge;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl2Front;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl3Front;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl4Front;
import com.stuypulse.robot.commands.froggy.FroggyAlgaeGroundIntake;
import com.stuypulse.robot.commands.froggy.FroggyCoralGroundIntake;
import com.stuypulse.robot.commands.froggy.FroggyProcessorScore;
import com.stuypulse.robot.commands.froggy.FroggyScoreL1;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.lokishooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.lokishooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.lokishooter.ShooterShootFront;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.funnel.CoralFunnel;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        funnel.setDefaultCommand(new FunnelDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

    /* RIGHT SIDE BUTTONS */
    // ADD ALIGNMENT TO ALL SCORING ROUTINES

    // BOTTOM BUTTON -> LVL 2 FRONT
    driver.getBottomButton()
        .whileTrue(new SequentialCommandGroup(
            new ElevatorToLvl2Front()
            .andThen(new ArmMoveToL2Front())
            .andThen(new ShooterShootFront()))); 
    
    // RIGHT BUTTON -> LVL 3 FRONT
    driver.getRightButton()
        .whileTrue(new SequentialCommandGroup(
            new ElevatorToLvl3Front()
            .andThen(new ArmMoveToL3Front())
            .andThen(new ShooterShootFront()))); 
    
    // TOP BUTTON -> LVL 4 FRONT
    driver.getTopButton()
        .whileTrue(new SequentialCommandGroup(
            new ElevatorToLvl4Front()
            .andThen(new ArmMoveToL4Front())
            .andThen(new ShooterShootFront()))); 

    // LEFT BUTTON -> BARGE SCORE
    driver.getLeftButton()
        .whileTrue(new SequentialCommandGroup(
            new ElevatorToBarge()
            .andThen(new ArmMoveToBarge()
            .andThen(new ShooterShootAlgae()))));
    
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
            new ElevatorToLvl3Front()
            .andThen(new ArmMoveToL3Front())
            .andThen(new ShooterAcquireAlgae()))); // left top paddle -> get algae
        
    // LEFT BOTTOM PADDLE -> L2 REEF ALGAE INTAKE
    driver.getDPadDown()
        .whileTrue(new SequentialCommandGroup(
            new ElevatorToLvl2Front()
            .andThen(new ArmMoveToL2Front())
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
