/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.commands.arm.ArmToAngle;
import com.stuypulse.robot.commands.arm.front_side.ArmToL2Front;
import com.stuypulse.robot.commands.arm.front_side.ArmToL3Front;
import com.stuypulse.robot.commands.arm.front_side.ArmToL4Front;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climb.*;
import com.stuypulse.robot.commands.elevator.*;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToBarge;
import com.stuypulse.robot.commands.elevator.front_side.ElevatorToL2Front;
import com.stuypulse.robot.commands.elevator.front_side.ElevatorToL3Front;
import com.stuypulse.robot.commands.elevator.front_side.ElevatorToL4Front;
import com.stuypulse.robot.commands.froggy.*;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.led.LedRainbow;
import com.stuypulse.robot.commands.led.LedSolidColor;
import com.stuypulse.robot.commands.lokishooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.lokishooter.ShooterShootFront;
import com.stuypulse.robot.commands.routines.*;
import com.stuypulse.robot.commands.routines.algae.*;
import com.stuypulse.robot.commands.routines.front_side.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.ArmImpl;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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

        new Trigger(() -> froggy.isCoralStalling() || shooter.hasCoral())
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
        driver.getBottomButton().onTrue(new ScoreL2Front()).onFalse(new MoveToFeed());

        // RIGHT BUTTON -> LVL 3 FRONT
        driver.getRightButton().whileTrue(new ScoreL3Front()).onFalse(new MoveToFeed());

        // TOP BUTTON -> LVL 4 FRONT
        driver.getTopButton().whileTrue(new ScoreL4Front()).onFalse(new MoveToFeedReverse());

        // LEFT BUTTON -> BARGE SCORE
        driver.getLeftButton().whileTrue(new ScoreBarge()).onFalse(new MoveToFeed());

        // TOP RIGHT PADDLE -> CLIMB OPEN
        driver.getLeftMenuButton().onTrue(new ClimbOpen());

        // BOTTOM RIGHT PADDLE -> SCORE (IN GENERAL)
        if (Arm.getInstance().getTargetAngle() == Settings.Arm.BARGE_ANGLE) {
            driver.getDPadRight().whileTrue(new ShooterShootAlgae());
        }
        else if (Froggy.getInstance().getTargetAngle() == Settings.Froggy.L1_SCORING_ANGLE) {
            driver.getDPadRight().whileTrue(new FroggyOuttakeCoral());
        }
        else if (Froggy.getInstance().isAlgaeStalling()) {
            driver.getDPadRight().whileTrue(new FroggyOuttakeAlgae());
        }
        else if (Arm.getInstance().getTargetAngle() == Settings.Arm.L2_ANGLE_FRONT || Arm.getInstance().getTargetAngle() == Settings.Arm.L3_ANGLE_FRONT || Arm.getInstance().getTargetAngle() == Settings.Arm.L4_ANGLE_FRONT){ 
            driver.getDPadRight().whileTrue(new ShooterShootFront());
        }
        // RIGHT MENU BUTTON -> CLIMB DRIVE
        driver.getRightMenuButton().onTrue(new ClimbClimb());

        // TOP LEFT PADDLE -> L3 REEF ALGAE INTAKE
        driver.getDPadLeft().whileTrue(new AcquireAlgaeL3()).onFalse(new MoveToStow());

        // BOTTOM LEFT PADDLE -> L2 REEF ALGAE INTAKE
        driver.getDPadDown().whileTrue(new AcquireAlgaeL2()).onFalse(new MoveToStow());

        // RIGHT BUMPER -> TO L1
        driver.getRightBumper().onTrue(new FroggyToL1());

        // RIGHT TRIGGER -> GROUND CORAL INTAKE
        driver.getRightTriggerButton().whileTrue(new FroggyCoralGroundIntake());

        // LEFT BUMPER -> PROCESSOR SCORE
        driver.getLeftBumper().whileTrue(new FroggyProcessorScore());

        // LEFT TRIGGER -> GROUND ALGAE INTAKE
        driver.getLeftTriggerButton().whileTrue(new FroggyAlgaeGroundIntake());


        /* OPERATOR BUTTON BINDINGS */

        // BOTTOM BUTTON -> GO TO LVL 2 HEIGHT
        operator.getBottomButton().whileTrue(
            new SequentialCommandGroup(
                new ElevatorToL2Front()
                .andThen(new ArmToL2Front()))
                    ).onFalse(new MoveToFeed()
            );

        // RIGHT BUTTON -> GO TO LVL 3 HEIGHT
        operator.getBottomButton().whileTrue(
            new SequentialCommandGroup(
                new ElevatorToL3Front()
                .andThen(new ArmToL3Front()))
                    ).onFalse(new MoveToFeed()
            );

        // TOP BUTTON -> GO TO LVL 4 HEIGHT
        operator.getBottomButton().whileTrue(
            new SequentialCommandGroup(
                new ElevatorToL4Front()
                .andThen(new ArmToL4Front()))
                    ).onFalse(new MoveToFeedReverse()
            );

        // LEFT BUTTON -> ELEVATOR TO BARGE
        operator.getLeftButton().whileTrue(new ElevatorToBarge()).onFalse(new ElevatorToFeed());

        // LEFT MENU BUTTON -> CLIMB DOWN
        operator.getLeftMenuButton().onTrue(new ManualClimb(Settings.Climb.MANUAL_CLIMB_SPEED)); //change, need a manual climb

        // RIGHT MENU BUTTON -> CLIMB UP
        operator.getRightMenuButton().onTrue(new ManualClimb(-Settings.Climb.MANUAL_CLIMB_SPEED));

        // BOTTOM RIGHT PADDLE -> SCORE (IN GENERAL)
        operator.getDPadRight().onTrue(new ArmToAngle(Rotation2d.fromDegrees(arm.getCurrentAngle().getDegrees() + 5)));

        // TOP LEFT PADDLE -> L3 REEF ALGAE INTAKE
        operator.getDPadLeft().onTrue(new ArmToAngle(Rotation2d.fromDegrees(arm.getCurrentAngle().getDegrees() - 5)));

        // 
        operator.getDPadUp().onTrue(new ElevatorToHeight(elevator.getCurrentHeight() - Units.inchesToMeters(2)));

        // BOTTOM LEFT PADDLE -> L2 REEF ALGAE INTAKE
        operator.getDPadDown().onTrue(new ElevatorToHeight(elevator.getCurrentHeight() + Units.inchesToMeters(2)));

        // RIGHT BUMPER -> TO L1
        operator.getRightBumper().whileTrue(new FroggyToL1());

        // RIGHT TRIGGER -> GROUND CORAL INTAKE
        operator.getRightTriggerButton().whileTrue(new FroggyCoralGroundIntake());

        // LEFT BUMPER -> PROCESSOR SCORE
        operator.getLeftBumper().whileTrue(new FroggyProcessorScore());

        // LEFT TRIGGER -> GROUND ALGAE INTAKE
        operator.getLeftTriggerButton().whileTrue(new FroggyAlgaeGroundIntake());
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
