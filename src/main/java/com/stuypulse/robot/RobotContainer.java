/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.ScoreRoutine;
import com.stuypulse.robot.commands.autons.FDCB.FourPieceFDCB;
import com.stuypulse.robot.commands.autons.FDCB.ThreeHalfPieceFDC;
import com.stuypulse.robot.commands.autons.FDCB.ThreePieceFDC;
import com.stuypulse.robot.commands.autons.GAlgae.OneGOneAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OneGThreeAlgae;
import com.stuypulse.robot.commands.autons.GAlgae.OneGTwoAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHOneAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHThreeAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHTwoAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OnePieceH;
import com.stuypulse.robot.commands.autons.IKLA.FourPieceIKLA;
import com.stuypulse.robot.commands.autons.IKLA.ThreeHalfPieceIKL;
import com.stuypulse.robot.commands.autons.IKLA.ThreePieceIKL;
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
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterUnjambCoralBackwards;
import com.stuypulse.robot.commands.superStructure.SuperStructureClimb;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureUnstuckCoral;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureBarge118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureProcessor;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge118AllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge118OppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilClearFromBarge118;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ClearAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ClearOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ScoreAllianceSide;
import com.stuypulse.robot.commands.swerve.driveAligned.barge118.SwerveDriveDriveAlignedToBarge118ScoreOppositeAllianceSide;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestCoralStation;
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

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
        configureDefaultCommands();
        configureAutomaticCommands();
        configureDriverButtonBindings();
        configureAutons();
        //configureSysids();

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
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(
                    new FroggyRollerShootCoral(),
                    new ConditionalCommand(
                        new ShooterShootAlgae(),
                        new FroggyRollerShootAlgae(), 
                        () -> superStructure.getState() == SuperStructureState.PROCESSOR),
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
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerHoldAlgae());

        // Froggy pivot to processor
        driver.getLeftBumper()
            .onTrue(new FroggyPivotToProcessor())
            .onTrue(new SuperStructureProcessor()
                .onlyIf(() -> !shooter.hasCoral()));

        // Ground coral intake and send elevator/arm to feed
        driver.getRightTriggerButton()
            .onTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.CORAL_GROUND_PICKUP)
                .andThen(new FroggyPivotToCoralGroundPickup().alongWith(new FroggyRollerIntakeCoral())))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()))
            .onFalse(new FroggyRollerHoldCoral());

        // L1
        driver.getRightBumper()
            .onTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE).andThen(new FroggyPivotToL1()).onlyIf(() -> !shooter.hasCoral()))
            .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE) && !shooter.hasCoral()))
            .onTrue(new SuperStructureCoralL1().onlyIf(() -> shooter.hasCoral()));

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
        // driver.getRightButton()
        //     .whileTrue(new ConditionalCommand(
        //         new ScoreRoutine(driver, 3, true).alongWith(new WaitUntilCommand(() -> false)),
        //         new ScoreRoutine(driver, 3, false).alongWith(new WaitUntilCommand(() -> false)),
        //         () -> swerve.isFrontFacingAllianceReef()))
        //     .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
        //         .andThen(new SuperStructureFeed()))
        //     .onFalse(new ShooterStop());

        // // L2 Coral Score
        // driver.getBottomButton()
        //     .whileTrue(new ConditionalCommand(
        //         new ScoreRoutine(driver, 2, true).alongWith(new WaitUntilCommand(() -> false)),
        //         new ScoreRoutine(driver, 2, false).alongWith(new WaitUntilCommand(() -> false)), 
        //         () -> swerve.isFrontFacingAllianceReef()))
        //     .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
        //         .andThen(new SuperStructureFeed()))
        //     .onFalse(new ShooterStop());

        // // Barge catapult score and align to coral station
        // driver.getLeftButton()
        //     .whileTrue(new ConditionalCommand(
        //         new ConditionalCommand(
        //             new SwerveDriveDriveAlignedToCatapultAllianceSide(driver)
        //                 .deadlineFor(new LEDApplyPattern(Settings.LED.ALIGN_COLOR))
        //                 .alongWith(new SuperStructureCatapultReady()
        //                     .andThen(new SuperStructureWaitUntilAtTarget()
        //                         .alongWith(new SwerveDriveWaitUntilAlignedToCatapultAllianceSide()))
        //                     .andThen(new SuperStructureCatapultShoot()
        //                         .andThen(new SuperStructureWaitUntilCanCatapult()
        //                             .andThen(new ShooterShootAlgae())))), 
        //             new SwerveDriveDriveAlignedToCatapultOppositeAllianceSide(driver)
        //                 .deadlineFor(new LEDApplyPattern(Settings.LED.ALIGN_COLOR))
        //                 .alongWith(new SuperStructureCatapultReady()
        //                     .andThen(new SuperStructureWaitUntilAtTarget()
        //                         .alongWith(new SwerveDriveWaitUntilAlignedToCatapultOppositeAllianceSide()))
        //                     .andThen(new SuperStructureCatapultShoot()
        //                         .andThen(new SuperStructureWaitUntilCanCatapult()
        //                             .andThen(new ShooterShootAlgae())))), 
        //             () -> swerve.getPose().getX() <= Field.LENGTH / 2
        //         ),
        //         new SwerveDrivePIDAssistToClosestCoralStation(driver),
        //         () -> shooter.getState() == ShooterState.HOLD_ALGAE
        //     ))
        //     .onFalse(new SuperStructureFeed())
        //     .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));
        
        // // Barge 118 style score and align to coral station
        driver.getLeftButton()
            .whileTrue(new ConditionalCommand(
                new ConditionalCommand(
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
                ),
                new SwerveDrivePIDAssistToClosestCoralStation(driver),
                () -> shooter.getState() == ShooterState.HOLD_ALGAE
            ))
            .onFalse(new SwerveDriveWaitUntilClearFromBarge118().andThen(new SuperStructureFeed()))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));

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
            .onTrue(new ShooterAcquireAlgae())
            .whileTrue(new ConditionalCommand(
                new SwerveDrivePidToNearestReefAlgae(true)
                    .alongWith(new SuperStructureAlgaeL3Front())
                    .andThen(new SwerveDriveNudgeForward()),
                new SwerveDrivePidToNearestReefAlgae(true)
                    .alongWith(new SuperStructureAlgaeL2Front())
                    .andThen(new SwerveDriveNudgeForward()), 
                () -> ReefUtil.getClosestAlgae().isHighAlgae()))
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
            .onTrue(new ShooterUnjambCoralBackwards().onlyIf(() -> climb.getState() == ClimbState.CLOSED))
            .onFalse(new ClimbIdle())
            .onFalse(new ShooterStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        swerve.configureAutoBuilder();

        /** TOP AUTONS **/

        AutonConfig BLUE_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Blue Mid Top to H");
        AutonConfig RED_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Red Mid Top to H");

        AutonConfig BLUE_THREE_PIECE_IKL = new AutonConfig("3 Piece IKL", ThreePieceIKL::new,
        "Blue I to HP", "Blue K to HP");
        AutonConfig RED_THREE_PIECE_IKL = new AutonConfig("3 Piece IKL", ThreePieceIKL::new,
        "Red I to HP", "Red K to HP");

        AutonConfig BLUE_THREE_HALF_PIECE_IKL = new AutonConfig("3.5 Piece IKL", ThreeHalfPieceIKL::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP");
        AutonConfig RED_THREE_HALF_PIECE_IKL = new AutonConfig("3.5 Piece IKL", ThreeHalfPieceIKL::new,
        "Red I to HP", "Red K to HP", "Red L to HP");

        AutonConfig BLUE_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA", FourPieceIKLA::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        AutonConfig RED_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA", FourPieceIKLA::new,
        "Red I to HP", "Red K to HP", "Red L to HP", "Red A BackOut");
        
        BLUE_ONE_PIECE_H.registerBlue(autonChooser);
        RED_ONE_PIECE_H.registerRed(autonChooser);

        BLUE_THREE_PIECE_IKL.registerBlue(autonChooser);
        RED_THREE_PIECE_IKL.registerRed(autonChooser);

        BLUE_THREE_HALF_PIECE_IKL.registerBlue(autonChooser);
        RED_THREE_HALF_PIECE_IKL.registerRed(autonChooser);

        BLUE_FOUR_PIECE_IKLA.registerDefaultBlue(autonChooser);
        RED_FOUR_PIECE_IKLA.registerDefaultRed(autonChooser);

        /** BOTTOM AUTONS **/

        // AutonConfig BLUE_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        // "Blue Mid Bottom to G");
        // AutonConfig RED_ONE_PIECE_G = new AutonConfig("1 Piece G", OnePieceG::new,
        // "Red Mid Bottom to G");

        AutonConfig BLUE_THREE_PIECE_FDC = new AutonConfig("3 Piece FDC", ThreePieceFDC::new,
        "Blue F to HP", "Blue D to HP");
        AutonConfig RED_THREE_PIECE_FDC = new AutonConfig("3 Piece FDC", ThreePieceFDC::new,
        "Red F to HP", "Red D to HP");

        AutonConfig BLUE_THREE_HALF_PIECE_FDC = new AutonConfig("3.5 Piece FDC", ThreeHalfPieceFDC::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP");
        AutonConfig RED_THREE_HALF_PIECE_FDC = new AutonConfig("3.5 Piece FDC", ThreeHalfPieceFDC::new,
        "Red F to HP", "Red D to HP", "Red C to HP");

        AutonConfig BLUE_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB", FourPieceFDCB::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        AutonConfig RED_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB", FourPieceFDCB::new,
        "Red F to HP", "Red D to HP", "Red C to HP", "Red B BackOut");

        //BLUE_ONE_PIECE_G.registerBlue(autonChooser);
        //RED_ONE_PIECE_G.registerRed(autonChooser);

        BLUE_THREE_PIECE_FDC.registerBlue(autonChooser);
        RED_THREE_PIECE_FDC.registerRed(autonChooser);

        BLUE_THREE_HALF_PIECE_FDC.registerBlue(autonChooser);
        RED_THREE_HALF_PIECE_FDC.registerRed(autonChooser);

        BLUE_FOUR_PIECE_FDCB.registerBlue(autonChooser);
        RED_FOUR_PIECE_FDCB.registerRed(autonChooser);

        /**  TOP ALGAE AUTONS **/

        AutonConfig BLUE_H_ONE_ALGAE = new AutonConfig("1 Piece H + 1 Algae", OneHOneAlgae::new,
        "Blue H BackOut", "Blue GH BackOut");
        AutonConfig RED_H_ONE_ALGAE = new AutonConfig("1 Piece H + 1 Algae", OneHOneAlgae::new,
        "Red H BackOut", "Red GH BackOut");

        AutonConfig BLUE_H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        "Blue H BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut");
        AutonConfig RED_H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        "Red H BackOut", "Red GH BackOut", "Red Barge to IJ (1)", "Red IJ BackOut");

        AutonConfig BLUE_H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        "Blue H BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut", "Blue Barge to EF (1)", "Blue EF BackOut");
        AutonConfig RED_H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae", OneHThreeAlgae::new,
        "Red H BackOut", "Red GH BackOut", "Red Barge to IJ (1)", "Red IJ BackOut", "Red Barge to EF (1)", "Red EF BackOut");

        BLUE_H_ONE_ALGAE.registerBlue(autonChooser);
        RED_H_ONE_ALGAE.registerRed(autonChooser);

        BLUE_H_TWO_ALGAE.registerBlue(autonChooser);
        RED_H_TWO_ALGAE.registerRed(autonChooser);

        BLUE_H_THREE_ALGAE.registerBlue(autonChooser);
        RED_H_THREE_ALGAE.registerRed(autonChooser);

        // /** BOTTOM ALGAE AUTONS **/

        AutonConfig BLUE_G_ONE_ALGAE = new AutonConfig("1 Piece G + 1 Algae", OneGOneAlgae::new,
        "Blue G BackOut", "Blue GH BackOut");
        AutonConfig RED_G_ONE_ALGAE = new AutonConfig("1 Piece G + 1 Algae", OneGOneAlgae::new,
        "Red G BackOut", "Red GH BackOut");

        AutonConfig BLUE_G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        "Blue G BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut");
        AutonConfig RED_G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        "Red G BackOut", "Red GH BackOut", "Red Barge to IJ (1)", "Red IJ BackOut");

        AutonConfig BLUE_G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        "Blue G BackOut", "Blue GH BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut", "Blue Barge to EF (1)", "Blue EF BackOut");
         AutonConfig RED_G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae", OneGThreeAlgae::new,
        "Red G BackOut", "Red GH BackOut", "Red Barge to IJ (1)", "Red IJ BackOut", "Red Barge to EF (1)", "Red EF BackOut");

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
