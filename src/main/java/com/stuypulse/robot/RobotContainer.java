/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.ScoreRoutine;
import com.stuypulse.robot.commands.arm.ArmOverrideVoltage;
import com.stuypulse.robot.commands.arm.ArmSetMotionProfileConstraints;
import com.stuypulse.robot.commands.arm.ArmToClimb;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmUnstuckCoral;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL2;
import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL3;
import com.stuypulse.robot.commands.arm.algae.ArmToBarge118;
import com.stuypulse.robot.commands.arm.algae.ArmToCatapultReady;
import com.stuypulse.robot.commands.arm.algae.ArmToCatapultShoot;
import com.stuypulse.robot.commands.arm.algae.ArmToProcessor;
import com.stuypulse.robot.commands.arm.algae.ArmWaitUntilCanCatapult;
import com.stuypulse.robot.commands.arm.coral.ArmToL1;
import com.stuypulse.robot.commands.arm.coral.ArmToL2Back;
import com.stuypulse.robot.commands.arm.coral.ArmToL2Front;
import com.stuypulse.robot.commands.arm.coral.ArmToL3Back;
import com.stuypulse.robot.commands.arm.coral.ArmToL3Front;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Back;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.autons.misc.DoNothingAuton;
import com.stuypulse.robot.commands.autons.misc.Mobility;
import com.stuypulse.robot.commands.autons.FDCB.FourPieceFDCB;
import com.stuypulse.robot.commands.autons.FDCB.FunnelFourPieceFDCB;
import com.stuypulse.robot.commands.autons.FDCB.ThreeHalfPieceFDC;
import com.stuypulse.robot.commands.autons.FDCB.ThreePieceFDC;
import com.stuypulse.robot.commands.autons.GAlgae.OneGOneAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OnePieceG;
import com.stuypulse.robot.commands.autons.HAlgae.OnePieceH;
import com.stuypulse.robot.commands.autons.IKLA.FourPieceIKLA;
import com.stuypulse.robot.commands.autons.IKLA.FunnelFourPieceIKLA;
import com.stuypulse.robot.commands.autons.IKLA.ThreeHalfPieceIKL;
import com.stuypulse.robot.commands.autons.IKLA.ThreePieceIKL;
import com.stuypulse.robot.commands.autons.tests.CurvyLineTest;
import com.stuypulse.robot.commands.autons.tests.RSquareTest;
import com.stuypulse.robot.commands.autons.tests.SquareTest;
import com.stuypulse.robot.commands.autons.tests.StraightLineTest;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbClose;
import com.stuypulse.robot.commands.climb.ClimbIdle;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbOverrideVoltage;
import com.stuypulse.robot.commands.climb.ClimbShimmy;
import com.stuypulse.robot.commands.elevator.ElevatorOverrideVoltage;
import com.stuypulse.robot.commands.elevator.ElevatorSetMotionProfileConstraints;
import com.stuypulse.robot.commands.elevator.ElevatorToClimb;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorToUnstuckCoral;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL2;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL3;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToBarge;
import com.stuypulse.robot.commands.elevator.algae.ElevatorToProcessor;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL1;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL2Back;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL2Front;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL3Back;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL3Front;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Back;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToClimb;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToCoralGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToProcessor;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToStow;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilAtTargetAngle;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilCanMoveWithoutColliding;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerHoldAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerHoldCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerStop;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.funnel.FunnelReverse;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.leds.LEDDisable;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterShootL1;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBargeAllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBargeOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBargeClearAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBargeClearOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBargeScoreAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBargeScoreOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveWaitUntilClearFromBarge;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToProcessorFroggy;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToProcessorShooter;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestCoralStation;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyReady;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyScore;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag1;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag2;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        leds.setDefaultCommand(new LEDDefaultCommand());
        shooter.setDefaultCommand(new ShooterAcquireCoral()
            .andThen(new BuzzController(driver))
            .onlyIf(() -> !shooter.hasCoral() 
                && shooter.getState() != ShooterState.ACQUIRE_ALGAE
                && shooter.getState() != ShooterState.HOLD_ALGAE 
                && shooter.getState() != ShooterState.SHOOT_ALGAE 
                && shooter.getState() != ShooterState.SHOOT_CORAL_L1
                && shooter.getState() != ShooterState.SHOOT_CORAL_FORWARD
                && shooter.getState() != ShooterState.SHOOT_CORAL_REVERSE
                && climb.getState() != ClimbState.OPEN
                && climb.getState() != ClimbState.CLIMBING));
    }

    private void configureAutomaticCommands() {
        RobotModeTriggers.disabled().and(() -> vision.getMaxTagCount() >= Settings.LED.DESIRED_TAGS_WHEN_DISABLED)
            .whileTrue(new LEDApplyPattern(Settings.LED.DISABLED_ALIGNED).ignoringDisable(true));

        RobotModeTriggers.disabled().and(() -> vision.getMaxTagCount() < Settings.LED.DESIRED_TAGS_WHEN_DISABLED)
            .debounce(0.5)
            .whileTrue(new LEDApplyPattern(LEDPattern.kOff).ignoringDisable(true));

        RobotModeTriggers.disabled()
            .onTrue(new VisionSetMegaTag1())
            .onFalse(new VisionSetMegaTag2());

        RobotModeTriggers.autonomous()
            .onTrue(new ElevatorSetMotionProfileConstraints(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND_AUTON, Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_AUTON))
            .onTrue(new ArmSetMotionProfileConstraints(Settings.Arm.MAX_VEL_AUTON, Settings.Arm.MAX_ACCEL_AUTON));

        RobotModeTriggers.teleop()
            .onTrue(new ElevatorSetMotionProfileConstraints(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND_TELEOP, Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_TELEOP))
            .onTrue(new ArmSetMotionProfileConstraints(Settings.Arm.MAX_VEL_TELEOP, Settings.Arm.MAX_ACCEL_TELEOP));
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureDriverButtonBindings() {

        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        driver.getDPadRight()
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(
                    new FroggyRollerShootAlgae().onlyIf(() -> froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE)
                        .alongWith(new ShooterShootAlgae().onlyIf(() -> arm.getState() == ArmState.PROCESSOR)), 
                    new FroggyRollerShootCoral(), 
                    () -> froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE), 
                new ConditionalCommand(
                    new ShooterShootAlgae(),
                    new ConditionalCommand(
                        new ShooterShootBackwards(),
                        new ConditionalCommand(
                            new ShooterShootL1(), 
                            new ShooterShootForwards(), 
                            () -> arm.getState() == ArmState.L1),
                        shooter::shouldShootBackwards
                    ), 
                    () -> shooter.getState() == ShooterState.HOLD_ALGAE), 
                () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE 
                    || froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE
                    || arm.getState() == ArmState.PROCESSOR))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE))
            .onFalse(new ConditionalCommand(
                new ArmToFeed().onlyIf(() -> arm.getState() == ArmState.PROCESSOR)
                    .alongWith(new ElevatorToFeed().onlyIf(() -> elevator.getState() == ElevatorState.PROCESSOR)), 
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new ElevatorToFeed().alongWith(new ArmToFeed()))
                    .onlyIf(() -> arm.getState() == ArmState.L4_FRONT
                        || arm.getState() == ArmState.L4_BACK
                        || arm.getState() == ArmState.L3_FRONT
                        || arm.getState() == ArmState.L3_BACK
                        || arm.getState() == ArmState.L2_FRONT
                        || arm.getState() == ArmState.L2_BACK
                        || arm.getState() == ArmState.L1), 
                () -> arm.getState() == ArmState.PROCESSOR || elevator.getState() == ElevatorState.PROCESSOR))
            .onFalse(new FroggyRollerStop()
                .onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL && froggy.getRollerState() != RollerState.HOLD_ALGAE))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()));

        // ground algae intake and reset
        driver.getLeftTriggerButton()
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .onTrue(new FroggyRollerIntakeAlgae())
            .onTrue(new ElevatorToFeed())
            .onTrue(new ArmToFeed())
            .onTrue(new ShooterStop()) // Exit algae hold state
            .onTrue(new ClimbClose())
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerHoldAlgae());

        // Froggy pivot to processor
        driver.getLeftBumper()
            .onTrue(new FroggyPivotToProcessor())
            .onTrue(new ElevatorToProcessor().alongWith(new ArmToProcessor())
                .onlyIf(() -> !shooter.hasCoral()));

        // Ground coral intake and send elevator/arm to feed
        driver.getRightTriggerButton()
            .onTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.CORAL_GROUND_PICKUP)
                .andThen(new FroggyPivotToCoralGroundPickup().alongWith(new FroggyRollerIntakeCoral())))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()))
            .onFalse(new FroggyRollerHoldCoral());

        // L1; if held, scores Loki L1; if tapped, scores Froggy L1
        driver.getRightBumper()
            .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE) && !shooter.hasCoral()))
            .whileTrue(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new ArmToL1().alongWith(new ElevatorToL1()))
                .onlyIf(() -> shooter.hasCoral()))
            .whileTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE)
                .andThen(new FroggyPivotToL1())
                .onlyIf(() -> !shooter.hasCoral()))
            .onFalse(new FroggyPivotToStow().alongWith(new FroggyRollerStop()).onlyIf(() -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE && froggy.getRollerState() == RollerState.SHOOT_CORAL))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_CORAL_L1));
        
        driver.getRightBumper().debounce(0.25)
            .whileTrue(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)
                .until(() -> shooter.getState() == ShooterState.SHOOT_CORAL_L1)
                .andThen(new LEDApplyPattern(Settings.LED.SCORE_COLOR)))
            .whileTrue(
                new WaitUntilCommand(() -> froggy.getCurrentAngle().getDegrees() > PivotState.L1_SCORE_ANGLE.getTargetAngle().getDegrees() - 10)
                    .deadlineFor(new SwerveDrivePIDToClosestL1FroggyReady())
                    .andThen(new SwerveDrivePIDToClosestL1FroggyScore()
                        .andThen(new FroggyRollerShootCoral()))
                    .onlyIf(() -> !shooter.hasCoral())
                )
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new ArmToFeed().alongWith(new ElevatorToFeed())).onlyIf(() -> arm.getState() == ArmState.L1 && arm.atTargetAngle() && elevator.getState() == ElevatorState.L1 && elevator.atTargetHeight()));

        // L4 coral score
        driver.getTopButton()
            .whileTrue(new ConditionalCommand(
                new ScoreRoutine(driver, 4, true, ArmState.L4_FRONT, ElevatorState.L4_FRONT).alongWith(new WaitUntilCommand(() -> false)),
                new ScoreRoutine(driver, 4, false, ArmState.L4_BACK, ElevatorState.L4_BACK).alongWith(new WaitUntilCommand(() -> false)),
                () -> swerve.isFrontFacingReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new ArmToFeed().alongWith(new ElevatorToFeed())))
            .onFalse(new ShooterStop());

        // L3 coral score
        driver.getRightButton()
            .whileTrue(new ConditionalCommand(
                new ScoreRoutine(driver, 3, true, ArmState.L3_FRONT, ElevatorState.L3_FRONT).alongWith(new WaitUntilCommand(() -> false)),
                new ScoreRoutine(driver, 3, false, ArmState.L3_BACK, ElevatorState.L3_BACK).alongWith(new WaitUntilCommand(() -> false)),
                () -> swerve.isFrontFacingReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new ArmToFeed().alongWith(new ElevatorToFeed())))
            .onFalse(new ShooterStop());

        // L2 coral score
        driver.getBottomButton()
            .whileTrue(new ConditionalCommand(
                new ScoreRoutine(driver, 2, true, ArmState.L2_FRONT, ElevatorState.L2_FRONT).alongWith(new WaitUntilCommand(() -> false)),
                new ScoreRoutine(driver, 2, false, ArmState.L2_BACK, ElevatorState.L2_BACK).alongWith(new WaitUntilCommand(() -> false)),
                () -> swerve.isFrontFacingReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new ArmToFeed().alongWith(new ElevatorToFeed())))
            .onFalse(new ShooterStop());

        // Barge score
        // driver.getLeftButton()
        //     .whileTrue(new ConditionalCommand(
        //         new ArmToBarge118().alongWith(new ElevatorToBarge())
        //         .andThen(new WaitUntilCommand(() -> arm.atTargetAngle() && elevator.atTargetHeight()).alongWith(new SwerveDriveWaitUntilClearFromBarge()))
        //             .deadlineFor(new SwerveDriveDriveAlignedToBargeClearAllianceSide(driver))
        //         .andThen(new SwerveDriveDriveAlignedToBargeScoreAllianceSide(driver))
        //         .andThen(new SwerveDriveWaitUntilAlignedToBargeAllianceSide())
        //         .andThen(new ShooterShootAlgae()), 
        //         // new ArmToBarge118().alongWith(new ElevatorToBarge())
        //         //     // .andThen(new WaitUntilCommand(() -> arm.atTargetAngle() && elevator.atTargetHeight()).alongWith(new SwerveDriveWaitUntilClearFromBarge()))
        //         //     .andThen(new SwerveDriveDriveAlignedToBargeClearAllianceSide(driver))
        //         //         // .deadlineFor(new SwerveDriveDriveAlignedToBargeClearAllianceSide(driver))
        //         //     .deadlineFor(new WaitUntilCommand(() -> arm.atTargetAngle() && elevator.atTargetHeight()).alongWith(new SwerveDriveWaitUntilClearFromBarge()))
        //         //     .andThen(new SwerveDriveDriveAlignedToBargeScoreAllianceSide(driver))
        //         //     .andThen(new SwerveDriveWaitUntilAlignedToBargeAllianceSide())
        //         //     .andThen(new ShooterShootAlgae()),    
                
        //         new ArmToBarge118().alongWith(new ElevatorToBarge())
        //             .andThen(new WaitUntilCommand(() -> arm.atTargetAngle() && elevator.atTargetHeight()).alongWith(new SwerveDriveWaitUntilClearFromBarge()))
        //                 .deadlineFor(new SwerveDriveDriveAlignedToBargeClearOppositeAllianceSide(driver))
        //             .andThen(new SwerveDriveDriveAlignedToBargeScoreOppositeAllianceSide(driver))
        //             .andThen(new SwerveDriveWaitUntilAlignedToBargeOppositeAllianceSide())
        //             .andThen(new ShooterShootAlgae()), 
                
        //         () -> swerve.getPose().getX() < Field.LENGTH / 2)
        //     ) 
        //     .onFalse(new SwerveDriveWaitUntilClearFromBarge().andThen(new ArmToFeed()).alongWith(new ElevatorToFeed()))
        //     .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));

        // CATAPULTING (HVR REVERT)
        driver.getLeftButton()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveDriveAlignedToBargeScoreAllianceSide(driver)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR))
                    .alongWith(new ElevatorToBarge().alongWith(new ArmToCatapultReady())
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())
                            .alongWith(new SwerveDriveWaitUntilAlignedToBargeAllianceSide()))
                        .andThen(new ArmToCatapultShoot()
                            .andThen(new ArmWaitUntilCanCatapult()
                                .andThen(new ShooterShootAlgae())))), 
                new SwerveDriveDriveAlignedToBargeScoreOppositeAllianceSide(driver)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR))
                    .alongWith(new ElevatorToBarge().alongWith(new ArmToCatapultReady())
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())
                            .alongWith(new SwerveDriveWaitUntilAlignedToBargeOppositeAllianceSide()))
                        .andThen(new ArmToCatapultShoot()
                            .andThen(new ArmWaitUntilCanCatapult()
                                .andThen(new ShooterShootAlgae())))), 
                () -> swerve.getPose().getX() <= Field.LENGTH / 2))
            .onFalse(new ElevatorToFeed().alongWith(new ArmToFeed()))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));

        // Acquire Closest Reef Algae
        driver.getDPadLeft()
            .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.isArmClearFromReef()))
            .whileTrue(new ConditionalCommand(
                new SwerveDrivePidToNearestReefAlgae()
                    .alongWith(new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3()))
                    .alongWith(new ShooterAcquireAlgae())
                        .andThen(new SwerveDriveNudgeForward()), 
                new SwerveDrivePidToNearestReefAlgae()
                    .alongWith(new ElevatorToAlgaeL2().alongWith(new ArmToAlgaeL2()))
                    .alongWith(new ShooterAcquireAlgae())
                        .andThen(new SwerveDriveNudgeForward()), 
                () -> ReefUtil.getClosestAlgae().isHighAlgae())
                .onlyIf(() -> Clearances.isArmClearFromReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new ElevatorToProcessor().alongWith(new ArmToProcessor())))
            .onFalse(new ShooterHoldAlgae());

        driver.getRightStickButton()
            .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
            .whileTrue(new SwerveDrivePIDAssistToClosestCoralStation(driver)
                .alongWith(new LEDApplyPattern(Settings.LED.CORAL_STATION_ALIGN_COLOR))
                .onlyIf(() -> !shooter.hasCoral()));

        // Unstuck Coral
        driver.getDPadDown()
            .onTrue(new ConditionalCommand(
                new ClimbShimmy(),
                new ElevatorToUnstuckCoral().alongWith(new ArmUnstuckCoral()),
                () -> climb.getState() == ClimbState.OPEN || climb.getState() == ClimbState.IDLE || climb.getState() == ClimbState.CLIMBING
            ));

        // Get ready for climb
        driver.getLeftMenuButton()
            .onTrue(new FroggyPivotToStow())
            .onTrue(new ElevatorToClimb().alongWith(new ArmToClimb())
                .andThen(new WaitUntilCommand(() -> elevator.atTargetHeight() || arm.atTargetAngle()))
                .andThen(new ClimbOpen()
                    .alongWith(new ShooterStop())
                    .alongWith(new FroggyRollerStop())));

        // Climb!!
        driver.getRightMenuButton()
            .onTrue(new ClimbClimb()
                .onlyIf(() -> climb.getState() == ClimbState.OPEN 
                    || climb.getState() == ClimbState.SHIMMY 
                    || climb.getState() == ClimbState.IDLE))
            .onTrue(new ShooterShootBackwards().onlyIf(() -> climb.getState() == ClimbState.CLOSED))
            .onFalse(new ClimbIdle().onlyIf(() -> climb.getState() == ClimbState.CLIMBING))
            .onFalse(new ShooterStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        swerve.configureAutoBuilder();

        /** TOP AUTONS **/

        // AutonConfig BLUE_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        // "Blue Mid Top to H");
        // AutonConfig RED_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        // "Red Mid Top to H");

        // AutonConfig BLUE_THREE_PIECE_IKL = new AutonConfig("3 Piece IKL", ThreePieceIKL::new,
        // "Blue I to HP", "Blue K to HP");
        // AutonConfig RED_THREE_PIECE_IKL = new AutonConfig("3 Piece IKL", ThreePieceIKL::new,
        // "Red I to HP", "Red K to HP");

        // AutonConfig BLUE_THREE_HALF_PIECE_IKL = new AutonConfig("3.5 Piece IKL", ThreeHalfPieceIKL::new,
        // "Blue I to HP", "Blue K to HP", "Blue L to HP");
        // AutonConfig RED_THREE_HALF_PIECE_IKL = new AutonConfig("3.5 Piece IKL", ThreeHalfPieceIKL::new,
        // "Red I to HP", "Red K to HP", "Red L to HP");

        AutonConfig BLUE_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA", FourPieceIKLA::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        AutonConfig RED_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA", FourPieceIKLA::new,
        "Red I to HP", "Red K to HP", "Red L to HP", "Red A BackOut");

        AutonConfig BLUE_FUNNEL_FOUR_PIECE_IKLA = new AutonConfig("4 Piece Funnel IKLA", FunnelFourPieceIKLA::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        AutonConfig RED_FUNNEL_FOUR_PIECE_IKLA = new AutonConfig("4 Piece Funnel IKLA", FunnelFourPieceIKLA::new,
        "Red I to HP", "Red K to HP", "Red L to HP", "Red A BackOut");
        
        // BLUE_ONE_PIECE_H.registerBlue(autonChooser);
        // RED_ONE_PIECE_H.registerRed(autonChooser);

        // BLUE_THREE_PIECE_IKL.registerBlue(autonChooser);
        // RED_THREE_PIECE_IKL.registerRed(autonChooser);

        // BLUE_THREE_HALF_PIECE_IKL.registerBlue(autonChooser);
        // RED_THREE_HALF_PIECE_IKL.registerRed(autonChooser);

        BLUE_FOUR_PIECE_IKLA.registerDefaultBlue(autonChooser);
        RED_FOUR_PIECE_IKLA.registerDefaultRed(autonChooser);

        BLUE_FUNNEL_FOUR_PIECE_IKLA.registerBlue(autonChooser);
        RED_FUNNEL_FOUR_PIECE_IKLA.registerRed(autonChooser);

        /** BOTTOM AUTONS **/

        // AutonConfig BLUE_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        // "Blue Mid Bottom to G");
        // AutonConfig RED_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        // "Red Mid Bottom to G");

        // AutonConfig BLUE_THREE_PIECE_FDC = new AutonConfig("3 Piece FDC", ThreePieceFDC::new,
        // "Blue F to HP", "Blue D to HP");
        // AutonConfig RED_THREE_PIECE_FDC = new AutonConfig("3 Piece FDC", ThreePieceFDC::new,
        // "Red F to HP", "Red D to HP");

        // AutonConfig BLUE_THREE_HALF_PIECE_FDC = new AutonConfig("3.5 Piece FDC", ThreeHalfPieceFDC::new,
        // "Blue F to HP", "Blue D to HP", "Blue C to HP");
        // AutonConfig RED_THREE_HALF_PIECE_FDC = new AutonConfig("3.5 Piece FDC", ThreeHalfPieceFDC::new,
        // "Red F to HP", "Red D to HP", "Red C to HP");

        AutonConfig BLUE_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB", FourPieceFDCB::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        AutonConfig RED_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB", FourPieceFDCB::new,
        "Red F to HP", "Red D to HP", "Red C to HP", "Red B BackOut");

        AutonConfig BLUE_FUNNEL_FOUR_PIECE_FDCB = new AutonConfig("4 Piece Funnel FDCB", FunnelFourPieceFDCB::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        AutonConfig RED_FUNNEL_FOUR_PIECE_FDCB = new AutonConfig("4 Piece Funnel FDCB", FunnelFourPieceFDCB::new,
        "Red F to HP", "Red D to HP", "Red C to HP", "Red B BackOut");

        // BLUE_ONE_PIECE_G.registerBlue(autonChooser);
        // RED_ONE_PIECE_G.registerRed(autonChooser);

        // BLUE_THREE_PIECE_FDC.registerBlue(autonChooser);
        // RED_THREE_PIECE_FDC.registerRed(autonChooser);

        // BLUE_THREE_HALF_PIECE_FDC.registerBlue(autonChooser);
        // RED_THREE_HALF_PIECE_FDC.registerRed(autonChooser);

        BLUE_FOUR_PIECE_FDCB.registerBlue(autonChooser);
        RED_FOUR_PIECE_FDCB.registerRed(autonChooser);

        BLUE_FUNNEL_FOUR_PIECE_FDCB.registerBlue(autonChooser);
        RED_FUNNEL_FOUR_PIECE_FDCB.registerRed(autonChooser);

        /**  TOP ALGAE AUTONS **/

        // AutonConfig BLUE_H_ONE_ALGAE = new AutonConfig("1 Piece H + 1 Algae", OneHOneAlgae::new,
        // "Blue Mid Top to H", "Blue H to GH Algae", "Blue GH Algae to Barge 3");
        // AutonConfig RED_H_ONE_ALGAE = new AutonConfig("1 Piece H + 1 Algae", OneHOneAlgae::new,
        // "Red Mid Top to H", "Red H to GH Algae", "Red GH Algae to Barge 3");

        // AutonConfig BLUE_H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        // "Blue Mid Top to H", "Blue H to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3");
        // AutonConfig RED_H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        // "Red Mid Top to H", "Red H to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3");

        // AutonConfig BLUE_H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        // "Blue Mid Top to H", "Blue H to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3", "Blue Barge 3 to EF Algae", "Blue EF Algae to Barge 3");
        // AutonConfig RED_H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        // "Red Mid Top to H", "Red H to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3", "Red Barge 3 to EF Algae", "Red EF Algae to Barge 3");

        // BLUE_H_ONE_ALGAE.registerBlue(autonChooser);
        // RED_H_ONE_ALGAE.registerRed(autonChooser);

        // BLUE_H_TWO_ALGAE.registerBlue(autonChooser);
        // RED_H_TWO_ALGAE.registerRed(autonChooser);

        // BLUE_H_THREE_ALGAE.registerBlue(autonChooser);
        // RED_H_THREE_ALGAE.registerRed(autonChooser);

        // /** BOTTOM ALGAE AUTONS **/

        // AutonConfig BLUE_G_ONE_ALGAE = new AutonConfig("1 Piece G + 1 Algae", OneGOneAlgae::new,
        // "Blue G to GH Algae");
        // AutonConfig RED_G_ONE_ALGAE = new AutonConfig("1 Piece G + 1 Algae", OneGOneAlgae::new,
        // "Red G to GH Algae");

        // AutonConfig BLUE_G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        // "Blue Mid Bottom to G", "Blue G to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3");
        // AutonConfig RED_G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        // "Red Mid Bottom to G", "Red G to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3");

        // AutonConfig BLUE_G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        // "Blue Mid Bottom to G", "Blue G to GH Algae", "Blue GH Algae to Barge 3", "Blue Barge 3 to IJ Algae", "Blue IJ Algae to Barge 3", "Blue Barge 3 to EF Algae", "Blue EF Algae to Barge 3");
         //AutonConfig RED_G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        // "Red Mid Bottom to G", "Red G to GH Algae", "Red GH Algae to Barge 3", "Red Barge 3 to IJ Algae", "Red IJ Algae to Barge 3", "Red Barge 3 to EF Algae", "Red EF Algae to Barge 3");

        // BLUE_G_ONE_ALGAE.registerBlue(autonChooser);
        // RED_G_ONE_ALGAE.registerRed(autonChooser);

        // BLUE_G_TWO_ALGAE.registerBlue(autonChooser);
        // RED_G_TWO_ALGAE.registerRed(autonChooser);

        // BLUE_G_THREE_ALGAE.registerBlue(autonChooser);
         //RED_G_THREE_ALGAE.registerRed(autonChooser);

        /** TESTS **/

        
        AutonConfig BLUE_MOBILITY = new AutonConfig("Mobility", Mobility::new,
        "Mobility");
        AutonConfig RED_MOBILITY = new AutonConfig("Mobility", Mobility::new, 
        "Mobility");
        /* 
        AutonConfig STRAIGHT_LINE_TEST = new AutonConfig("Straight Line Test", StraightLineTest::new,
        "Straight Line");
        AutonConfig CURVY_LINE_TEST = new AutonConfig("Curvy Line Test", CurvyLineTest::new,
        "Curvy Line");
        AutonConfig SQUARE_TEST = new AutonConfig("Square Test", SquareTest::new,
        "Square Top", "Square Right", "Square Bottom", "Square Left");
        AutonConfig RSQUARE_TEST = new AutonConfig("RSquare Test", RSquareTest::new,
        "RSquare Top", "RSquare Right", "RSquare Bottom", "RSquare Left");
        */
        BLUE_MOBILITY.registerBlue(autonChooser);
        RED_MOBILITY.registerRed(autonChooser);
        /*
        STRAIGHT_LINE_TEST.registerRed(autonChooser);
        CURVY_LINE_TEST.registerRed(autonChooser);
        SQUARE_TEST.registerRed(autonChooser);
        RSQUARE_TEST.registerRed(autonChooser);
        */

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

        SysIdRoutine froggyPivotSysIdRoutine = froggy.getPivotSysIdRoutine();
        autonChooser.addOption("Froggy Pivot Quasi Forward", froggyPivotSysIdRoutine.quasistatic(Direction.kForward));
        autonChooser.addOption("Froggy Pivot Quasi Backward", froggyPivotSysIdRoutine.quasistatic(Direction.kReverse));
        autonChooser.addOption("Froggy Pivot Dynamic Forward", froggyPivotSysIdRoutine.dynamic(Direction.kForward));
        autonChooser.addOption("Froggy Pivot Dynamic Backward", froggyPivotSysIdRoutine.dynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
