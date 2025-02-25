/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.arm.ArmOffsetTargetDown;
import com.stuypulse.robot.commands.arm.ArmOffsetTargetUp;
import com.stuypulse.robot.commands.arm.ArmOverrideVoltage;
import com.stuypulse.robot.commands.arm.ArmToClimb;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL2;
import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL3;
import com.stuypulse.robot.commands.arm.algae.ArmToBarge;
import com.stuypulse.robot.commands.arm.algae.ArmToHoldAlgae;
import com.stuypulse.robot.commands.arm.algae.ArmToProcessor;
import com.stuypulse.robot.commands.arm.coral.ArmToL2Back;
import com.stuypulse.robot.commands.arm.coral.ArmToL2Front;
import com.stuypulse.robot.commands.arm.coral.ArmToL3Back;
import com.stuypulse.robot.commands.arm.coral.ArmToL3Front;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Back;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.autons.EDCB.FourPieceEDCB;
import com.stuypulse.robot.commands.autons.EDCB.OnePieceE;
import com.stuypulse.robot.commands.autons.EDCB.ThreeHalfPieceEDC;
import com.stuypulse.robot.commands.autons.EDCB.ThreePieceEDC;
import com.stuypulse.robot.commands.autons.EDCB.TwoPieceED;
import com.stuypulse.robot.commands.autons.JKLA.FourPieceJKLA;
import com.stuypulse.robot.commands.autons.JKLA.OnePieceJ;
import com.stuypulse.robot.commands.autons.JKLA.ThreeHalfPieceJKL;
import com.stuypulse.robot.commands.autons.JKLA.ThreePieceJKL;
import com.stuypulse.robot.commands.autons.JKLA.TwoPieceJK;
import com.stuypulse.robot.commands.autons.misc.DoNothingAuton;
import com.stuypulse.robot.commands.autons.misc.Mobility;
import com.stuypulse.robot.commands.autons.GAlgae.OneGOneAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OneGThreeAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OneGTwoAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OnePieceG;
import com.stuypulse.robot.commands.autons.HAlgae.OneHOneAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHThreeAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHTwoAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OnePieceH;
import com.stuypulse.robot.commands.autons.tests.CurvyLineTest;
import com.stuypulse.robot.commands.autons.tests.RSquareTest;
import com.stuypulse.robot.commands.autons.tests.SquareTest;
import com.stuypulse.robot.commands.autons.tests.StraightLineTest;
import com.stuypulse.robot.commands.autons.tests.testing;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbIdle;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbOverrideVoltage;
import com.stuypulse.robot.commands.elevator.ElevatorOffsetTargetDown;
import com.stuypulse.robot.commands.elevator.ElevatorOffsetTargetUp;
import com.stuypulse.robot.commands.elevator.ElevatorOverrideVoltage;
import com.stuypulse.robot.commands.elevator.ElevatorToClimb;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToHoldAlgae;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToProcessor;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL2;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL3;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToBarge;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL2Back;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL2Front;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL3Back;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL3Front;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Back;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotMoveOperatorOffsetDown;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotMoveOperatorOffsetUp;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToCoralGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToProcessor;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToStow;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilAtTargetAngle;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerStop;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.funnel.FunnelReverse;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedToBarge;
import com.stuypulse.robot.commands.swerve.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
import com.stuypulse.robot.commands.swerve.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge;
import com.stuypulse.robot.commands.vision.VisionSetIMUMode;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag1;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag2;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final Telemetry telemetry = new Telemetry(Settings.Swerve.Constraints.MAX_VELOCITY.get());
    private final LimelightVision vision = LimelightVision.getInstance();
    private final Funnel funnel = Funnel.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Arm arm = Arm.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Froggy froggy = Froggy.getInstance();
    private final LEDController leds = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        configureDefaultCommands();
        configureAutomaticCommands();
        configureDriverButtonBindings();
        configureOperatorButtonBindings();
        configureAutons();
        configureSysids();

        // swerve.registerTelemetry(telemetry::telemeterize);
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULT ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        funnel.setDefaultCommand(new FunnelDefaultCommand());
        leds.setDefaultCommand(new LEDDefaultCommand().ignoringDisable(true));
        shooter.setDefaultCommand(new ShooterAcquireCoral()
            .andThen(new BuzzController(driver))
            .onlyIf(() -> !shooter.hasCoral() 
                && shooter.getState() != ShooterState.ACQUIRE_ALGAE
                && shooter.getState() != ShooterState.HOLD_ALGAE 
                && shooter.getState() != ShooterState.SHOOT_ALGAE 
                && shooter.getState() != ShooterState.SHOOT_CORAL_FORWARD
                && shooter.getState() != ShooterState.SHOOT_CORAL_REVERSE));
    }

    private void configureAutomaticCommands() {
        RobotModeTriggers.disabled().and(() -> vision.getMaxTagCount() > Settings.LED.DESIRED_TAGS_WHEN_DISABLED)
            .whileTrue(new LEDApplyPattern(Settings.LED.DISABLED_ALIGNED).ignoringDisable(true));

        RobotModeTriggers.disabled()
            .onTrue(new VisionSetMegaTag1())
            .onFalse(new VisionSetMegaTag2());
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureDriverButtonBindings() {

        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        // manual shoot depending on whatever states robot is in: either score barge, forwards/backwards on reef, or processor/L1
        driver.getDPadRight()
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(
                    new FroggyRollerShootAlgae(), 
                    new FroggyRollerShootCoral(), 
                    () -> froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE), 
                new ConditionalCommand(
                    new ShooterShootAlgae(), 
                    new ConditionalCommand(
                        new ShooterShootBackwards(),
                        new ShooterShootForwards(),
                        shooter::shouldShootBackwards
                    ), 
                    () -> arm.getState() == ArmState.BARGE), 
                () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE || froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE))
            .onFalse(new ShooterStop())
            .onFalse(new FroggyRollerStop());

        // ground algae intake and send elevator/arm to feed
        driver.getLeftTriggerButton()
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .onTrue(new FroggyRollerIntakeAlgae())
            .onTrue(new ElevatorToFeed())
            .onTrue(new ArmToFeed())
            .onTrue(new ShooterStop()) // Exit algae hold state
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerStop());

        // Algae processor score
        driver.getLeftBumper()
            .whileTrue(new FroggyPivotToProcessor()
                .andThen(new FroggyPivotWaitUntilAtTargetAngle())
                .andThen(new FroggyRollerShootAlgae()))
            .whileTrue(new ElevatorToProcessor().alongWith(new ArmToProcessor())
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                .andThen(new ShooterShootAlgae())
                .onlyIf(() -> !shooter.hasCoral()))
            .onFalse(new FroggyRollerStop())
            .onFalse(new ShooterStop());

        // Ground coral intake and send elevator/arm to feed
        driver.getRightTriggerButton()
            .onTrue(new FroggyPivotToCoralGroundPickup())
            .onTrue(new FroggyRollerIntakeCoral())
            .onTrue(new ElevatorToFeed())
            .onTrue(new ArmToFeed())
            .onTrue(new ShooterStop()) // Exit algae hold state
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerStop());

        // L1 coral score
        driver.getRightBumper()
            .whileTrue(new FroggyPivotToL1()
                .andThen(new FroggyPivotWaitUntilAtTargetAngle())
                .andThen(new FroggyRollerShootCoral()))
            .onFalse(new FroggyRollerStop())
            .onFalse(new FroggyPivotToStow());

        // L4 coral score
        driver.getTopButton()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveCoralScoreAlignWithClearance(4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT)
                    .alongWith(new WaitUntilCommand(() -> swerve.isClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ElevatorToL4Front().alongWith(new ArmToL4Front()))
                        .onlyIf(() -> elevator.getState() != ElevatorState.L4_FRONT || arm.getState() != ArmState.L4_FRONT)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                    .andThen(new ShooterShootBackwards()),
                new SwerveDriveCoralScoreAlignWithClearance(4, false, ElevatorState.L4_BACK, ArmState.L4_BACK)
                    .alongWith(new WaitUntilCommand(() -> swerve.isClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ElevatorToL4Back().alongWith(new ArmToL4Back()))
                        .onlyIf(() -> elevator.getState() != ElevatorState.L4_BACK || arm.getState() != ArmState.L4_BACK)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                    .andThen(new ShooterShootForwards()), 
                () -> swerve.isFrontFacingReef()))
            .onFalse(new WaitUntilCommand(() -> swerve.isClearFromReef())
                .andThen(new ElevatorToFeed().alongWith(new ArmToFeed())))
            .onFalse(new ShooterStop());
                
        // L3 coral score
        driver.getRightButton()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveCoralScoreAlignWithClearance(3, true, ElevatorState.L3_FRONT, ArmState.L3_FRONT)
                    .alongWith(new WaitUntilCommand(() -> swerve.isClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ElevatorToL3Front().alongWith(new ArmToL3Front()))
                        .onlyIf(() -> elevator.getState() != ElevatorState.L3_FRONT || arm.getState() != ArmState.L3_FRONT)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                    .andThen(new ShooterShootBackwards()),
                new SwerveDriveCoralScoreAlignWithClearance(3, false, ElevatorState.L3_BACK, ArmState.L3_BACK)
                    .alongWith(new WaitUntilCommand(() -> swerve.isClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ElevatorToL3Back().alongWith(new ArmToL3Back()))
                        .onlyIf(() -> elevator.getState() != ElevatorState.L3_BACK || arm.getState() != ArmState.L3_BACK)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                    .andThen(new ShooterShootForwards()), 
                () -> swerve.isFrontFacingReef()))
            .onFalse(new WaitUntilCommand(() -> swerve.isClearFromReef())
                .andThen(new ElevatorToFeed().alongWith(new ArmToFeed())))
            .onFalse(new ShooterStop());

        // L2 coral score
        driver.getBottomButton()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveCoralScoreAlignWithClearance(2, true, ElevatorState.L2_FRONT, ArmState.L2_FRONT)
                    .alongWith(new WaitUntilCommand(() -> swerve.isClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ElevatorToL2Front().alongWith(new ArmToL2Front()))
                        .onlyIf(() -> elevator.getState() != ElevatorState.L2_FRONT || arm.getState() != ArmState.L2_FRONT)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                    .andThen(new ShooterShootForwards()),
                new SwerveDriveCoralScoreAlignWithClearance(2, false, ElevatorState.L2_BACK, ArmState.L2_BACK)
                    .alongWith(new WaitUntilCommand(() -> swerve.isClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ElevatorToL2Back().alongWith(new ArmToL2Back()))
                        .onlyIf(() -> elevator.getState() != ElevatorState.L2_BACK || arm.getState() != ArmState.L2_BACK)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                    .andThen(new ShooterShootForwards()), 
                () -> swerve.isFrontFacingReef()))
            .onFalse(new WaitUntilCommand(() -> swerve.isClearFromReef())
                .andThen(new ElevatorToFeed().alongWith(new ArmToFeed())))
            .onFalse(new ShooterStop());

        // Barge score
        driver.getLeftButton()
            .whileTrue(new SwerveDriveDriveAlignedToBarge(driver))
            .whileTrue(new ElevatorToBarge().alongWith(new ArmToBarge())
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())
                    .alongWith(new SwerveDriveWaitUntilAlignedToBarge()
                        .alongWith(new LEDApplyPattern(Settings.LED.ALIGN_COLOR))))
                .andThen(new ShooterShootAlgae()))
            .onFalse(new ElevatorToFeed().alongWith(new ArmToFeed()))
            .onFalse(new ShooterStop());

        // Acquire Closest Reef Algae
        driver.getDPadLeft()
            .whileTrue(new ConditionalCommand(
                new SwerveDrivePidToNearestReefAlgae()
                    .alongWith(new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3()))
                    .alongWith(new ShooterAcquireAlgae()), 
                new SwerveDrivePidToNearestReefAlgae()
                    .alongWith(new ElevatorToAlgaeL2().alongWith(new ArmToAlgaeL2()))
                    .alongWith(new ShooterAcquireAlgae()), 
                () -> ReefUtil.getClosestAlgae().isHighAlgae()))
            .onFalse(new WaitUntilCommand(() -> swerve.isClearFromReef())
                .andThen(new ElevatorToHoldAlgae().alongWith(new ArmToHoldAlgae())))
            .onFalse(new ShooterHoldAlgae());

        // Get ready for climb
        driver.getLeftMenuButton()
            .onTrue(new ElevatorToClimb().alongWith(new ArmToClimb())
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                .andThen(new ClimbOpen()));

        // Climb!!
        driver.getRightMenuButton()
            .onTrue(new ClimbClimb())
            .onFalse(new ClimbIdle());
    }

    private void configureOperatorButtonBindings() {
        // Elevator Voltage Override
        new Trigger(() -> Math.abs(operator.getLeftStick().y) > Settings.Operator.Elevator.VOLTAGE_OVERRIDE_DEADBAND)   
            .whileTrue(new ElevatorOverrideVoltage(() -> -operator.getLeftStick().y > 0 
                ? (-operator.getLeftStick().y * Math.abs(Settings.Operator.Elevator.MAX_VOLTAGE_UP))
                : (-operator.getLeftStick().y * Math.abs(Settings.Operator.Elevator.MAX_VOLTAGE_DOWN))));
        
        // Arm Voltage Override
        new Trigger(() -> Math.abs(operator.getRightStick().x) > Settings.Operator.Arm.VOLTAGE_OVERRIDE_DEADBAND)
            .whileTrue(new ArmOverrideVoltage(() -> operator.getRightStick().x > 0 
                ? (operator.getRightStick().x * Math.abs(Settings.Operator.Arm.MAX_VOLTAGE_UP))
                : (operator.getRightStick().x * Math.abs(Settings.Operator.Arm.MAX_VOLTAGE_DOWN))));

        // Shoot Backwards
        operator.getLeftTriggerButton()
            .whileTrue(new ConditionalCommand(
                new ShooterShootForwards().alongWith(new FunnelReverse()),
                new ShooterShootBackwards().alongWith(new FunnelReverse()),
                () -> shooter.shouldShootBackwards()))
            .onFalse(new ShooterStop());

        // Shoot Forwards
        operator.getRightTriggerButton()
            .whileTrue(new ConditionalCommand(
                new ShooterShootBackwards(), 
                new ShooterShootForwards(), 
                () -> shooter.shouldShootBackwards()))
            .onFalse(new ShooterStop());

        // Froggy pivot offsets
        operator.getLeftBumper().whileTrue(new FroggyPivotMoveOperatorOffsetUp());
        operator.getRightBumper().whileTrue(new FroggyPivotMoveOperatorOffsetDown());

        // Climb voltage overrides
        operator.getLeftMenuButton().whileTrue(new ClimbOverrideVoltage(Settings.Operator.Climb.CLIMB_DOWN_VOLTAGE));
        operator.getRightMenuButton().whileTrue(new ClimbOverrideVoltage(Settings.Operator.Climb.CLIMB_UP_VOLTAGE)); 

        // Arm/Elevator to heights
        operator.getTopButton().onTrue(swerve.isFrontFacingReef() 
            ? new ElevatorToL4Front().alongWith(new ArmToL4Front()) 
            : new ElevatorToL4Back().alongWith(new ArmToL4Back()));
        operator.getRightButton().onTrue(swerve.isFrontFacingReef() 
            ? new ElevatorToL3Front().alongWith(new ArmToL3Front()) 
            : new ElevatorToL3Back().alongWith(new ArmToL3Back()));
        operator.getBottomButton().onTrue(swerve.isFrontFacingReef() 
            ? new ElevatorToL2Front().alongWith(new ArmToL2Front()) 
            : new ElevatorToL2Back().alongWith(new ArmToL2Back()));

        operator.getLeftButton()
            .onTrue(new ElevatorToBarge())
            .onTrue(new ArmToBarge());

        // Elevator offsets
        operator.getDPadUp().onTrue(new ElevatorOffsetTargetUp());
        operator.getDPadDown().onTrue(new ElevatorOffsetTargetDown());

        // Arm offsets
        operator.getDPadLeft().onTrue(new ArmOffsetTargetDown());
        operator.getDPadRight().onTrue(new ArmOffsetTargetUp());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        swerve.configureAutoBuilder();

        AutonConfig testa = new AutonConfig("testa", testing::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Red C to HP", "Red HP to B");

        testa.registerRed(autonChooser);

        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        /** TOP AUTONS **/

        AutonConfig BLUE_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Blue Mid Top to H");
        AutonConfig RED_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Red Mid Top to H");

        AutonConfig BLUE_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePieceJ::new,
        "Blue Top to J", "Red I BackOut");
        AutonConfig RED_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePieceJ::new,
        "Red Top to J", "Red I BackOut");

        AutonConfig BLUE_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPieceJK::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K");
        AutonConfig RED_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPieceJK::new,
        "Red Top to J", "Red J to HP", "Red HP to K");

       // AutonConfig BLUE_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePieceJKL::new,
        //"Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Red I BackOut");
        AutonConfig RED_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePieceJKL::new,
        "Red J to HP", "Red K to HP", "Red I BackOut");

        AutonConfig BLUE_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPieceJKL::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP");
        AutonConfig RED_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPieceJKL::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Red L to HP");

        AutonConfig BLUE_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPieceJKLA::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP","Blue HP to A");
        AutonConfig RED_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPieceJKLA::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Red L to HP", "Red HP to A");
        
        BLUE_ONE_PIECE_H.registerBlue(autonChooser);
        RED_ONE_PIECE_H.registerRed(autonChooser);
       
        BLUE_ONE_PIECE_J.registerBlue(autonChooser);
        RED_ONE_PIECE_J.registerRed(autonChooser);

        BLUE_TWO_PIECE_JK.registerBlue(autonChooser);
        RED_TWO_PIECE_JK.registerRed(autonChooser);

       // BLUE_THREE_PIECE_JKL.registerBlue(autonChooser);
        RED_THREE_PIECE_JKL.registerRed(autonChooser);

        BLUE_THREE_HALF_PIECE_JKL.registerBlue(autonChooser);
        RED_THREE_HALF_PIECE_JKL.registerRed(autonChooser);

        BLUE_FOUR_PIECE_JKLA.registerBlue(autonChooser);
        RED_FOUR_PIECE_JKLA.registerRed(autonChooser);

        /** BOTTOM AUTONS **/

        AutonConfig BLUE_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        "Blue Mid Bottom to G");
        AutonConfig RED_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        "Red Mid Bottom to G");
        AutonConfig BLUE_ONE_PIECE_E = new AutonConfig("1 Piece E", OnePieceE::new,
        "Blue Bottom to E");
        AutonConfig RED_ONE_PIECE_E = new AutonConfig("1 Piece E", OnePieceE::new,
        "Red Bottom to E");

        AutonConfig BLUE_TWO_PIECE_ED = new AutonConfig("2 Piece ED", TwoPieceED::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D");
        AutonConfig RED_TWO_PIECE_ED = new AutonConfig("2 Piece ED", TwoPieceED::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D");

        AutonConfig BLUE_THREE_PIECE_EDC = new AutonConfig("3 Piece EDC", ThreePieceEDC::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C");
        AutonConfig RED_THREE_PIECE_EDC = new AutonConfig("3 Piece EDC", ThreePieceEDC::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C");

        AutonConfig BLUE_THREE_HALF_PIECE_EDC = new AutonConfig("3.5 Piece EDC", ThreeHalfPieceEDC::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP");
        AutonConfig RED_THREE_HALF_PIECE_EDC = new AutonConfig("3.5 Piece EDC", ThreeHalfPieceEDC::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Red C to HP");

        AutonConfig BLUE_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPieceEDCB::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP","Blue HP to B");
        AutonConfig RED_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPieceEDCB::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Red C to HP", "Red HP to B");

        BLUE_ONE_PIECE_G.registerBlue(autonChooser);
        RED_ONE_PIECE_G.registerRed(autonChooser);
       
        BLUE_ONE_PIECE_E.registerBlue(autonChooser);
        RED_ONE_PIECE_E.registerRed(autonChooser);

        BLUE_TWO_PIECE_ED.registerBlue(autonChooser);
        RED_TWO_PIECE_ED.registerRed(autonChooser);

        BLUE_THREE_PIECE_EDC.registerBlue(autonChooser);
        RED_THREE_PIECE_EDC.registerRed(autonChooser);

        BLUE_THREE_HALF_PIECE_EDC.registerBlue(autonChooser);
        RED_THREE_HALF_PIECE_EDC.registerRed(autonChooser);

        BLUE_FOUR_PIECE_EDCB.registerBlue(autonChooser);
        RED_FOUR_PIECE_EDCB.registerRed(autonChooser);

        /**  TOP ALGAE AUTONS **/

        AutonConfig BLUE_H_ONE_ALGAE = new AutonConfig("1 Piece H + 1 Algae", OneHOneAlgae::new,
        "Blue Mid Top to H", "Blue H to GH Algae", "Blue GH Algae to Barge 3");
        AutonConfig RED_H_ONE_ALGAE = new AutonConfig("1 Piece H + 1 Algae", OneHOneAlgae::new,
        "Red Mid Top to H", "Red H to GH Algae", "Red GH Algae to Barge 3");

        AutonConfig BLUE_H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        "Blue Mid Top to H", "Blue H to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3");
        AutonConfig RED_H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        "Red Mid Top to H", "Red H to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3");

        AutonConfig BLUE_H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        "Blue Mid Top to H", "Blue H to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3", "Blue Barge 3 to EF Algae", "Blue EF Algae to Barge 3");
        AutonConfig RED_H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        "Red Mid Top to H", "Red H to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3", "Red Barge 3 to EF Algae", "Red EF Algae to Barge 3");

        BLUE_H_ONE_ALGAE.registerBlue(autonChooser);
        RED_H_ONE_ALGAE.registerRed(autonChooser);

        BLUE_H_TWO_ALGAE.registerBlue(autonChooser);
        RED_H_TWO_ALGAE.registerRed(autonChooser);

        BLUE_H_THREE_ALGAE.registerBlue(autonChooser);
        RED_H_THREE_ALGAE.registerRed(autonChooser);

        /** BOTTOM ALGAE AUTONS **/

        AutonConfig BLUE_G_ONE_ALGAE = new AutonConfig("1 Piece G + 1 Algae", OneGOneAlgae::new,
        "Blue Mid Bottom to G", "Blue G to GH Algae", "Blue GH Algae to Barge 3");
        AutonConfig RED_G_ONE_ALGAE = new AutonConfig("1 Piece G + 1 Algae", OneGOneAlgae::new,
        "Red Mid Bottom to G", "Red G to GH Algae", "Red GH Algae to Barge 3");

        AutonConfig BLUE_G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        "Blue Mid Bottom to G", "Blue G to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3");
        AutonConfig RED_G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        "Red Mid Bottom to G", "Red G to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3");

        AutonConfig BLUE_G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        "Blue Mid Bottom to G", "Blue G to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3", "Blue Barge 3 to EF Algae", "Blue EF Algae to Barge 3");
        AutonConfig RED_G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        "Red Mid Bottom to G", "Red G to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3", "Red Barge 3 to EF Algae", "Red EF Algae to Barge 3");

        BLUE_G_ONE_ALGAE.registerBlue(autonChooser);
        RED_G_ONE_ALGAE.registerRed(autonChooser);

        BLUE_G_TWO_ALGAE.registerBlue(autonChooser);
        RED_G_TWO_ALGAE.registerRed(autonChooser);

        BLUE_G_THREE_ALGAE.registerBlue(autonChooser);
        RED_G_THREE_ALGAE.registerRed(autonChooser);

        /** TESTS **/

        AutonConfig BLUE_MOBILITY = new AutonConfig("Mobility", Mobility::new,
        "Mobility");
        AutonConfig RED_MOBILITY = new AutonConfig("Mobility", Mobility::new, 
        "Mobility");
        AutonConfig STRAIGHT_LINE_TEST = new AutonConfig("Straight Line Test", StraightLineTest::new,
        "Straight Line");
        AutonConfig CURVY_LINE_TEST = new AutonConfig("Curvy Line Test", CurvyLineTest::new,
        "Curvy Line");
        AutonConfig SQUARE_TEST = new AutonConfig("Square Test", SquareTest::new,
        "Square Top", "Square Right", "Square Bottom", "Square Left");
        AutonConfig RSQUARE_TEST = new AutonConfig("RSquare Test", RSquareTest::new,
        "RSquare Top", "RSquare Right", "RSquare Bottom", "RSquare Left");
        AutonConfig testPath = new AutonConfig("testing", testing::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Red C to HP", "Red HP to B");


        BLUE_MOBILITY.registerBlue(autonChooser);
        RED_MOBILITY.registerRed(autonChooser);
        STRAIGHT_LINE_TEST.registerRed(autonChooser);
        CURVY_LINE_TEST.registerRed(autonChooser);
        SQUARE_TEST.registerRed(autonChooser);
        RSQUARE_TEST.registerRed(autonChooser);
        testPath.registerRed(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public void configureSysids() {
        autonChooser.addOption("Swerve Quasi Forward", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("Swerve Quasi Backward", swerve.sysIdQuasistatic(Direction.kReverse));
        autonChooser.addOption("Swerve Dynamic Forward", swerve.sysIdDynamic(Direction.kForward));
        autonChooser.addOption("Swerve Dynamic Backward", swerve.sysIdDynamic(Direction.kReverse));

        SysIdRoutine elevatorSysIdRoutine = elevator.getSysIdRoutine();
        autonChooser.addOption("Elevator Quasi Forward", elevatorSysIdRoutine.quasistatic(Direction.kForward));
        autonChooser.addOption("Elevator Quasi Backward", elevatorSysIdRoutine.quasistatic(Direction.kReverse));
        autonChooser.addOption("Elevator Dynamic Forward", elevatorSysIdRoutine.dynamic(Direction.kForward));
        autonChooser.addOption("Elevator Dynamic Backward", elevatorSysIdRoutine.dynamic(Direction.kReverse));

        SysIdRoutine armSysIdRoutine = arm.getSysIdRoutine();
        autonChooser.addOption("Arm Quasi Forward", armSysIdRoutine.quasistatic(Direction.kForward));
        autonChooser.addOption("Arm Quasi Backward", armSysIdRoutine.quasistatic(Direction.kReverse));
        autonChooser.addOption("Arm Dynamic Forward", armSysIdRoutine.dynamic(Direction.kForward));
        autonChooser.addOption("Arm Dynamic Backward", armSysIdRoutine.dynamic(Direction.kReverse));

        SysIdRoutine froggyPivotSysIdRoutine = froggy.getFroggySysIdRoutine();
        autonChooser.addOption("Froggy Pivot Quasi Forward", froggyPivotSysIdRoutine.quasistatic(Direction.kForward));
        autonChooser.addOption("Froggy Pivot Quasi Backward", froggyPivotSysIdRoutine.quasistatic(Direction.kReverse));
        autonChooser.addOption("Froggy Pivot Dynamic Forward", froggyPivotSysIdRoutine.dynamic(Direction.kForward));
        autonChooser.addOption("Froggy Pivot Dynamic Backward", froggyPivotSysIdRoutine.dynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
