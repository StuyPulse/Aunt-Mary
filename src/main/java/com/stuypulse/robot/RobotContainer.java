/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.ReefAlgaePickupRoutine;
import com.stuypulse.robot.commands.ScoreRoutine;
import com.stuypulse.robot.commands.autons.FDCB.PathfulFourPieceFDCB;
import com.stuypulse.robot.commands.autons.FDCB.CheaterFourPieceFDCB;
import com.stuypulse.robot.commands.autons.FDCB.FourPieceFDCB;
import com.stuypulse.robot.commands.autons.GAlgae.OneGThreeAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OneGTwoAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHThreeAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHTwoAlgae;
import com.stuypulse.robot.commands.autons.IKLA.PathfulFourPieceIKLA;
import com.stuypulse.robot.commands.autons.IKLA.CheaterFourPieceIKLA;
import com.stuypulse.robot.commands.autons.IKLA.FourPieceIKLA;
import com.stuypulse.robot.commands.autons.misc.Mobility;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbClose;
import com.stuypulse.robot.commands.climb.ClimbIdle;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbShimmy;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToCoralGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToProcessor;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToStow;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilCanMoveWithoutColliding;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerHoldAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerHoldCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerStop;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBasedOnSuperStructure;
import com.stuypulse.robot.commands.shooter.ShooterShootL1;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterUnjamCoralBackwards;
import com.stuypulse.robot.commands.superStructure.SuperStructureClimb;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureUnstuckCoral;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureBarge118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureProcessor;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge118AllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge118OppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapultAllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapultOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilClearFromBarge118;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ClearAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ClearOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ScoreAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ScoreOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.catapult.SwerveDriveDriveAlignedToCatapultAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.catapult.SwerveDriveDriveAlignedToCatapultOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.pathFindToPose.SwerveDrivePathFindToPose;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestCoralStation;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestL1ShooterReady;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestL1ShooterScore;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyReady;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyScore;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag1;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag2;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;
import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    private final SuperStructure superStructure = SuperStructure.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Froggy froggy = Froggy.getInstance();
    private final LEDController leds = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        swerve.configureAutoBuilder();

        configureDefaultCommands();
        configureAutomaticCommands();
        configureDriverButtonBindings();
        configureAutons();
        // configureSysids();

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
                && shooter.getState() != ShooterState.UNJAMB_CORAL_BACKWARDS
                && !shooter.isShooting()
                && climb.getState() == ClimbState.CLOSED));
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
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureDriverButtonBindings() {

        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        // Manual Shoot
        driver.getDPadRight()
            .whileTrue(new LEDApplyPattern(Settings.LED.MANUAL_SHOOT_COLOR))
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(
                    new FroggyRollerShootCoral(),
                    new ShooterShootAlgae().onlyIf(() -> superStructure.getState() == SuperStructureState.PROCESSOR)
                        .alongWith(new FroggyRollerShootAlgae().onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL)),
                    () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE), 
                new ShooterShootBasedOnSuperStructure(),
                () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE 
                    || froggy.getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE
                    || superStructure.getState() == SuperStructureState.PROCESSOR))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE))
            .onFalse(new ConditionalCommand(
                new SuperStructureFeed().onlyIf(() -> superStructure.getState() == SuperStructureState.PROCESSOR), 
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureFeed())
                    .onlyIf(() -> superStructure.isScoringCoral()), 
                () -> superStructure.getState() == SuperStructureState.PROCESSOR))
            .onFalse(new FroggyRollerStop()
                .onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL && froggy.getRollerState() != RollerState.HOLD_ALGAE))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()));

        // ground algae intake and reset
        driver.getLeftTriggerButton()
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .onTrue(new FroggyRollerIntakeAlgae())
            .onTrue(new SuperStructureFeed())
            .onTrue(new ShooterStop()) // Exit algae hold state
            .onTrue(new ClimbClose())
            .whileTrue(new LEDApplyPattern(Settings.LED.INTAKE_COLOR_ALGAE))
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerHoldAlgae());

        // Processor
        driver.getLeftBumper()
            .onTrue(new FroggyPivotToProcessor()
                .onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL))
            .onTrue(new SuperStructureProcessor()
                .onlyIf(() -> !shooter.hasCoral()));

        // Ground coral intake and send elevator/arm to feed
        driver.getRightTriggerButton()
            .whileTrue(new LEDApplyPattern(Settings.LED.FROGGY_INTAKE_COLOR_CORAL))
            .onTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.CORAL_GROUND_PICKUP)
                .andThen(new FroggyPivotToCoralGroundPickup().alongWith(new FroggyRollerIntakeCoral())))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()))
            .onFalse(new FroggyRollerHoldCoral()); 

        // L1
        driver.getRightBumper()
            .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE) && !shooter.hasCoral()))
            .whileTrue(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new SuperStructureCoralL1())
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
            .whileTrue(new ConditionalCommand(
                new WaitUntilCommand(() -> superStructure.getState() == SuperStructureState.L1 && superStructure.atTarget())
                    .deadlineFor(new SwerveDrivePIDAssistToClosestL1ShooterReady(driver))
                    .andThen(new SwerveDrivePIDAssistToClosestL1ShooterScore(driver)
                        .alongWith(new WaitUntilCommand(() -> ReefUtil.getClosestReefFace().isAlignedToL1ShooterTarget())
                            .andThen(new ShooterShootL1()))),
                new WaitUntilCommand(() -> froggy.getCurrentAngle().getDegrees() > PivotState.L1_SCORE_ANGLE.getTargetAngle().getDegrees() - 10)
                    .deadlineFor(new SwerveDrivePIDToClosestL1FroggyReady())
                    .andThen(new SwerveDrivePIDToClosestL1FroggyScore()
                        .andThen(new FroggyRollerShootCoral())), 
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new SuperStructureFeed()).onlyIf(() -> superStructure.getState() == SuperStructureState.L1));

        // L4 Coral Score
        driver.getTopButton()
            .whileTrue(new ConditionalCommand(
                new ScoreRoutine(driver, 4, true).alongWith(new WaitUntilCommand(() -> false)),
                new ScoreRoutine(driver, 4, false).alongWith(new WaitUntilCommand(() -> false)),
                () -> swerve.isFrontFacingAllianceReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new SuperStructureFeed()))
            .onFalse(new ShooterStop());

        // L3 Coral Score
        driver.getRightButton()
            .whileTrue(new ConditionalCommand(
                new ScoreRoutine(driver, 3, true).alongWith(new WaitUntilCommand(() -> false)),
                new ScoreRoutine(driver, 3, false).alongWith(new WaitUntilCommand(() -> false)),
                () -> swerve.isFrontFacingAllianceReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new SuperStructureFeed()))
            .onFalse(new ShooterStop());

        // L2 Coral Score
        driver.getBottomButton()
            .whileTrue(new ConditionalCommand(
                new ScoreRoutine(driver, 2, true).alongWith(new WaitUntilCommand(() -> false)),
                new ScoreRoutine(driver, 2, false).alongWith(new WaitUntilCommand(() -> false)), 
                () -> swerve.isFrontFacingAllianceReef()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new SuperStructureFeed()))
            .onFalse(new ShooterStop());

        // // Barge catapult score
        // driver.getLeftButton()
        //     .whileTrue(new ConditionalCommand(
        //         new SwerveDriveDriveAlignedToCatapultAllianceSide(driver)
        //             .deadlineFor(new LEDApplyPattern(Settings.LED.ALIGN_COLOR))
        //             .alongWith(new SuperStructureCatapultReady()
        //                 .andThen(new SuperStructureWaitUntilAtTarget()
        //                     .alongWith(new SwerveDriveWaitUntilAlignedToCatapultAllianceSide()))
        //                 .andThen(new SuperStructureCatapultShoot()
        //                     .andThen(new SuperStructureWaitUntilCanCatapult()
        //                         .andThen(new ShooterShootAlgae())))), 
        //         new SwerveDriveDriveAlignedToCatapultOppositeAllianceSide(driver)
        //             .deadlineFor(new LEDApplyPattern(Settings.LED.ALIGN_COLOR))
        //             .alongWith(new SuperStructureCatapultReady()
        //                 .andThen(new SuperStructureWaitUntilAtTarget()
        //                     .alongWith(new SwerveDriveWaitUntilAlignedToCatapultOppositeAllianceSide()))
        //                 .andThen(new SuperStructureCatapultShoot()
        //                     .andThen(new SuperStructureWaitUntilCanCatapult()
        //                         .andThen(new ShooterShootAlgae())))), 
        //         () -> swerve.getPose().getX() <= Field.LENGTH / 2
        //     ))
        //     .onFalse(new SuperStructureFeed())
        //     .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));
        
        // Barge 118 style score
        driver.getLeftButton()
            .whileTrue(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)
                .until(() -> shooter.getState() == ShooterState.SHOOT_ALGAE)
                .andThen(new LEDApplyPattern(Settings.LED.SCORE_COLOR)))
            .whileTrue(new ConditionalCommand(
                new SuperStructureBarge118()
                    .andThen(new SwerveDriveWaitUntilClearFromBarge118().alongWith(new SuperStructureWaitUntilAtTarget())
                        .deadlineFor(new SwerveDriveDriveAlignedToBarge118ClearAllianceSide(driver))
                        .andThen(new SwerveDriveDriveAlignedToBarge118ScoreAllianceSide(driver)
                            .alongWith(new SwerveDriveWaitUntilAlignedToBarge118AllianceSide()
                                .andThen(new ShooterShootAlgae())))),
                new SuperStructureBarge118()
                    .andThen(new SwerveDriveWaitUntilClearFromBarge118().alongWith(new SuperStructureWaitUntilAtTarget())
                        .deadlineFor(new SwerveDriveDriveAlignedToBarge118ClearOppositeAllianceSide(driver))
                        .andThen(new SwerveDriveDriveAlignedToBarge118ScoreOppositeAllianceSide(driver)
                            .alongWith(new SwerveDriveWaitUntilAlignedToBarge118OppositeAllianceSide()
                                .andThen(new ShooterShootAlgae())))), 
                () -> swerve.getPose().getX() <= Field.LENGTH / 2
            ))
            .onFalse(new SwerveDriveWaitUntilClearFromBarge118().andThen(new SuperStructureFeed()))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));
        
        // Align to closest Coral Station
        driver.getRightStickButton()
            .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
            .whileTrue(SwerveDrivePathFindToPose.pathFindToNearestCoralStation()
                .until(() -> swerve.getPose().getX() < Field.ALLIANCE_REEF_CENTER.getX())
                .andThen(new SwerveDrivePIDAssistToClosestCoralStation(driver))
                .alongWith(new LEDApplyPattern(Settings.LED.CORAL_STATION_ALIGN_COLOR))
                .onlyIf(() -> !shooter.hasCoral()));

        // Acquire Closest Reef Algae
        // driver.getDPadLeft()
        //     .onTrue(new ShooterAcquireAlgae())
        //     .whileTrue(new ConditionalCommand(
        //         new ConditionalCommand(
        //             new SwerveDrivePidToNearestReefAlgae(true)
        //                 .alongWith(new SuperStructureAlgaeL3Front())
        //                 .andThen(new SwerveDriveNudgeForward()), 
        //             new SwerveDrivePidToNearestReefAlgae(false)
        //                 .alongWith(new SuperStructureAlgaeL3Back())
        //                 .andThen(new SwerveDriveNudgeBackwards()), 
        //             () -> (swerve.getPose().getX() < Field.LENGTH / 2 && swerve.isFrontFacingAllianceReef())
        //                 || (swerve.getPose().getX() > Field.LENGTH / 2 && swerve.isFrontFacingOppositeAllianceReef())),
        //         new ConditionalCommand(
        //             new SwerveDrivePidToNearestReefAlgae(true)
        //                 .alongWith(new SuperStructureAlgaeL2Front())
        //                 .andThen(new SwerveDriveNudgeForward()), 
        //             new SwerveDrivePidToNearestReefAlgae(false)
        //                 .alongWith(new SuperStructureAlgaeL2Back())
        //                 .andThen(new SwerveDriveNudgeBackwards()), 
        //             () -> (swerve.getPose().getX() < Field.LENGTH / 2 && swerve.isFrontFacingAllianceReef())
        //                 || (swerve.getPose().getX() > Field.LENGTH / 2 && swerve.isFrontFacingOppositeAllianceReef())), 
        //         () -> ReefUtil.getClosestAlgae().isHighAlgae()))
        //     .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
        //         .andThen(new SuperStructureProcessor()))
        //     .onFalse(new ShooterHoldAlgae());

        // Acquire closest reef algae front only
        driver.getDPadLeft()
            .whileTrue(new ReefAlgaePickupRoutine())
            .whileTrue(new LEDApplyPattern(Settings.LED.INTAKE_COLOR_ALGAE))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new SuperStructureProcessor()))
            .onFalse(new ShooterHoldAlgae());

        // Unstuck Coral and Climb Shimmy
        driver.getDPadDown()
            .onTrue(new ConditionalCommand(
                new ClimbShimmy(),
                new SuperStructureUnstuckCoral(),
                () -> climb.getState() != ClimbState.CLOSED
            ));

        // Get ready for climb
        driver.getLeftMenuButton()
            .onTrue(new FroggyPivotToStow())
            .onTrue(new SuperStructureClimb()
                .andThen(new WaitUntilCommand(() -> Elevator.getInstance().atTargetHeight() || Arm.getInstance().atTargetAngle()))
                .andThen(new ClimbOpen()
                    .alongWith(new ShooterStop())
                    .alongWith(new FroggyRollerStop())));

        // Climb!!
        driver.getRightMenuButton()
            .onTrue(new ClimbClimb()
                .onlyIf(() -> climb.getState() == ClimbState.OPEN 
                    || climb.getState() == ClimbState.SHIMMY 
                    || climb.getState() == ClimbState.IDLE))
            .onTrue(new ShooterUnjamCoralBackwards().onlyIf(() -> climb.getState() == ClimbState.CLOSED))
            .onFalse(new ClimbIdle().onlyIf(() -> climb.getState() == ClimbState.CLIMBING))
            .onFalse(new ShooterStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        /** TOP AUTONS **/

        AutonConfig FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA", FourPieceIKLA::new,
        "Blue I to HP");
        FOUR_PIECE_IKLA.registerBlue(autonChooser);

        AutonConfig CHEATER_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA (Cheater)", CheaterFourPieceIKLA::new,
         "Blue I to HP");
         CHEATER_FOUR_PIECE_IKLA.registerBlue(autonChooser);

        AutonConfig PATHFUL_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA (Pathful)", PathfulFourPieceIKLA::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        PATHFUL_FOUR_PIECE_IKLA.registerDefaultBlue(autonChooser);

        /** BOTTOM AUTONS **/

        AutonConfig FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB", FourPieceFDCB::new,
        "Blue F to HP");
        FOUR_PIECE_FDCB.registerBlue(autonChooser);

        AutonConfig CHEATER_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB (Cheater)", CheaterFourPieceFDCB::new,
         "Blue F to HP");
         CHEATER_FOUR_PIECE_FDCB.registerBlue(autonChooser);

        AutonConfig PATHFUL_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB (Pathful)", PathfulFourPieceFDCB::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        PATHFUL_FOUR_PIECE_FDCB.registerBlue(autonChooser);

        /**  TOP ALGAE AUTONS **/

        AutonConfig H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        "Blue H BackOut", "Blue Barge to IJ (1)", "Blue Barge BackOut");
        AutonConfig H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        "Blue H BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut", "Blue Barge to EF (1)", "Blue EF BackOut");
        H_TWO_ALGAE.registerBlue(autonChooser);
        H_THREE_ALGAE.registerBlue(autonChooser);

        // /** BOTTOM ALGAE AUTONS **/

        AutonConfig G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        "Blue G BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut");
        AutonConfig G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        "Blue G BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut", "Blue Barge to EF (1)", "Blue EF BackOut");
        G_TWO_ALGAE.registerBlue(autonChooser);
        G_THREE_ALGAE.registerBlue(autonChooser);

        /** TESTS **/

        AutonConfig MOBILITY = new AutonConfig("Mobility", Mobility::new,
        "Mobility");
        MOBILITY.registerBlue(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public void configureSysids() {
        autonChooser.addOption("Swerve Quasi Forward", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("Swerve Quasi Backward", swerve.sysIdQuasistatic(Direction.kReverse));
        autonChooser.addOption("Swerve Dynamic Forward", swerve.sysIdDynamic(Direction.kForward));
        autonChooser.addOption("Swerve Dynamic Backward", swerve.sysIdDynamic(Direction.kReverse));

        SysIdRoutine elevatorSysIdRoutine = Elevator.getInstance().getSysIdRoutine();
        autonChooser.addOption("Elevator Quasi Forward", elevatorSysIdRoutine.quasistatic(Direction.kForward));
        autonChooser.addOption("Elevator Quasi Backward", elevatorSysIdRoutine.quasistatic(Direction.kReverse));
        autonChooser.addOption("Elevator Dynamic Forward", elevatorSysIdRoutine.dynamic(Direction.kForward));
        autonChooser.addOption("Elevator Dynamic Backward", elevatorSysIdRoutine.dynamic(Direction.kReverse));

        SysIdRoutine armSysIdRoutine = Arm.getInstance().getSysIdRoutine();
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
