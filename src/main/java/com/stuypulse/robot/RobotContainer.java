/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToStow;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilAtTargetAngle;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerStop;
import com.stuypulse.robot.commands.funnel.FunnelAcquire;
import com.stuypulse.robot.commands.funnel.FunnelStop;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superstructure.SuperStructureToAlgaeL2;
import com.stuypulse.robot.commands.superstructure.SuperStructureToAlgaeL3;
import com.stuypulse.robot.commands.superstructure.SuperStructureToBarge;
import com.stuypulse.robot.commands.superstructure.SuperStructureToFeed;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL2Back;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL2Front;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL3Back;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL3Front;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL4Back;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL4Front;
import com.stuypulse.robot.commands.superstructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.SuperStructure;
import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.swerve.Telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final Telemetry telemetry = new Telemetry(Settings.Swerve.Constraints.MAX_VELOCITY.get());
    private final Funnel funnel = Funnel.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final SuperStructure superStructure = SuperStructure.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Froggy froggy = Froggy.getInstance();
    private final LEDController leds = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        swerve.registerTelemetry(telemetry::telemeterize);
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULT ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        leds.setDefaultCommand(new LEDDefaultCommand());
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureButtonBindings() {

        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        driver.getDPadRight()
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(
                    new ShooterShootAlgae(), 
                    new ConditionalCommand(
                        new ShooterShootForwards(), 
                        new ShooterShootBackwards().onlyIf(() -> shooter.shouldShootBackwards()), 
                        () -> shooter.shouldShootForward()),
                    () -> superStructure.getTargetState() == SuperStructureTargetState.BARGE), 
                new ConditionalCommand(
                    new FroggyRollerShootCoral(), 
                    new FroggyRollerShootAlgae().onlyIf(() -> froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE), 
                    () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE), 
                () -> superStructure.isInScoreState()));

        driver.getLeftTriggerButton()
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .whileTrue(new FroggyRollerIntakeAlgae())
            .onFalse(new FroggyPivotToStow());

        driver.getLeftBumper()
            .onTrue(new FroggyRollerShootAlgae())
            .onFalse(new FroggyRollerStop());

        driver.getRightTriggerButton()
            .onTrue(new SuperStructureToFeed())
            .whileTrue(new FunnelAcquire())
            .onFalse(new FunnelStop())
            .whileTrue(new ShooterAcquireCoral());

        driver.getRightBumper()
            .whileTrue(new FroggyPivotToL1()
                .andThen(new FroggyPivotWaitUntilAtTargetAngle())
                .andThen(new FroggyRollerShootCoral()))
            .onFalse(new FroggyRollerStop().andThen(new FroggyPivotToStow()));

        driver.getTopButton()
            .whileTrue(
                new ConditionalCommand(
                    new SuperStructureToL4Front()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(4, true)))
                        .andThen(new ShooterShootBackwards()), 
                    new SuperStructureToL4Back()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(4, false)))
                        .andThen(new ShooterShootForwards()), 
                    () -> swerve.isFrontFacingReef())
            )
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getRightButton()
            .whileTrue(
                new ConditionalCommand(
                    new SuperStructureToL3Front()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(3, true)))
                        .andThen(new ShooterShootBackwards()), 
                    new SuperStructureToL3Back()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(3, false)))
                        .andThen(new ShooterShootForwards()), 
                    () -> swerve.isFrontFacingReef())
            )
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getBottomButton()
            .whileTrue(
                new ConditionalCommand(
                    new SuperStructureToL2Front()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(2, true)))
                        .andThen(new ShooterShootForwards()), 
                    new SuperStructureToL2Back()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(2, false)))
                        .andThen(new ShooterShootForwards()), 
                    () -> swerve.isFrontFacingReef())
            )
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getLeftButton()
            .whileTrue(new SuperStructureToBarge()
                .andThen(new SuperStructureWaitUntilAtTarget())
                .andThen(new ShooterShootAlgae()))
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getDPadLeft()
            .whileTrue(new SuperStructureToAlgaeL3()
                .andThen(new ShooterAcquireAlgae()))
            .onFalse(new SuperStructureToFeed());
        
        driver.getDPadDown()
            .whileTrue(new SuperStructureToAlgaeL2()
                .andThen(new ShooterAcquireAlgae()))
            .onFalse(new SuperStructureToFeed());

        driver.getLeftMenuButton().onTrue(new ClimbOpen());
        driver.getRightMenuButton().onTrue(new ClimbClimb());
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
