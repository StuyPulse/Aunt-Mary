
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.ManualShoot;
import com.stuypulse.robot.commands.ReefAlgaePickupRoutineBack;
import com.stuypulse.robot.commands.ReefAlgaePickupRoutineFront;
import com.stuypulse.robot.commands.Reset;
import com.stuypulse.robot.commands.ScoreRoutine;
import com.stuypulse.robot.commands.autons.FDCB.FourPieceFDCB;
import com.stuypulse.robot.commands.autons.FDCB.FourPieceFDCE;
import com.stuypulse.robot.commands.autons.FDCB.FourPieceNudgeFDCE;
import com.stuypulse.robot.commands.autons.FDCB.PathfulFourPieceFDCB;
import com.stuypulse.robot.commands.autons.GAlgae.OneGTwoAlgae;
import com.stuypulse.robot.commands.autons.HAlgae.OneHTwoAlgae;
import com.stuypulse.robot.commands.autons.IKLA.FourPieceIKLA;
import com.stuypulse.robot.commands.autons.IKLA.FourPieceIKLJ;
import com.stuypulse.robot.commands.autons.IKLA.FourPieceNudgeIKLJ;
import com.stuypulse.robot.commands.autons.IKLA.PathfulFourPieceIKLA;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbClose;
import com.stuypulse.robot.commands.climb.ClimbIdle;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbShimmy;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToCoralGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToGolfTeeAlgaePickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1;
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
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureProcessor;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapult;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToCatapult;
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
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.robot.util.ReefUtil;

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
        configureDriverButtonBindings();
        configureAutons();
        // configureSysids();

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
                && shooter.getState() != ShooterState.UNJAM_CORAL_BACKWARDS
                && !shooter.isShooting()
                && climb.getState() == ClimbState.CLOSED));
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureDriverButtonBindings() {

        driver.getDPadUp().onTrue(new SwerveDriveResetRotation());

        // Manual Shoot
        driver.getDPadRight()
            .onTrue(new ManualShoot())
            .whileTrue(new LEDApplyPattern(Settings.LED.MANUAL_SHOOT_COLOR))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE))
            .onFalse(new ConditionalCommand(
                new SuperStructureFeed().onlyIf(() -> superStructure.getState() == SuperStructureState.PROCESSOR), 
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureFeed())
                    .onlyIf(() -> superStructure.isScoringCoral()), 
                () -> superStructure.getState() == SuperStructureState.PROCESSOR))
            .onFalse(new FroggyRollerStop()
                .onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL && froggy.getRollerState() != RollerState.HOLD_ALGAE))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow()));

        // ground algae intake and reset
        driver.getLeftTriggerButton()
            .onTrue(new Reset())
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .onTrue(new FroggyRollerIntakeAlgae())
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerHoldAlgae());

        // Froggy golf tee algae pickup
        driver.getLeftBumper()
            .onTrue(new FroggyPivotToGolfTeeAlgaePickup())
            .onTrue(new FroggyRollerIntakeAlgae())
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerHoldAlgae());

        // Ground coral intake and send elevator/arm to feed
        driver.getRightTriggerButton()
            .onTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.CORAL_GROUND_PICKUP)
                .andThen(new FroggyPivotToCoralGroundPickup().alongWith(new FroggyRollerIntakeCoral())))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()))
            .onFalse(new FroggyRollerHoldCoral()); 

        // L1
        driver.getRightBumper()
            .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE) && !shooter.hasCoral()))
            .whileTrue(new ConditionalCommand(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(new SuperStructureCoralL1()),
                new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE)
                    .andThen(new FroggyPivotToL1()), 
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow().alongWith(new FroggyRollerStop()))
                .onlyIf(() -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE && froggy.getRollerState() == RollerState.SHOOT_CORAL))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_CORAL_L1));
        
        driver.getRightBumper().debounce(0.25)
            .whileTrue(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)
                .until(() -> shooter.getState() == ShooterState.SHOOT_CORAL_L1))
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
        
        // Catapult
        driver.getLeftButton()
            .whileTrue(new SwerveDriveDriveAlignedToCatapult(driver)
                .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR))
                .alongWith(new SuperStructureCatapultReady()
                    .andThen(new SuperStructureWaitUntilAtTarget()
                        .alongWith(new SwerveDriveWaitUntilAlignedToCatapult()))
                    .andThen(new SuperStructureCatapultShoot()
                        .andThen(new SuperStructureWaitUntilCanCatapult()
                            .andThen(new ShooterShootAlgae())))))
            .onFalse(new SuperStructureFeed())
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));
        
        // Align to closest Coral Station
        // driver.getRightStickButton()
        //     .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
        //     .whileTrue(SwerveDrivePathFindToPose.pathFindToNearestCoralStation()
        //         .until(() -> swerve.getPose().getX() < Field.ALLIANCE_REEF_CENTER.getX())
        //         .andThen(new SwerveDrivePIDAssistToClosestCoralStation(driver))
        //         .alongWith(new LEDApplyPattern(Settings.LED.CORAL_STATION_ALIGN_COLOR))
        //         .onlyIf(() -> !shooter.hasCoral()));

        // Align to closest Coral Station without path finding
        driver.getRightStickButton()
            .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
            .whileTrue(new SwerveDrivePIDAssistToClosestCoralStation(driver)
                .alongWith(new LEDApplyPattern(Settings.LED.CORAL_STATION_ALIGN_COLOR))
                .onlyIf(() -> !shooter.hasCoral()));

        // Acquire closest reef algae
        driver.getDPadLeft()
            .whileTrue(new ConditionalCommand(
                new ReefAlgaePickupRoutineFront(),
                new ReefAlgaePickupRoutineBack(),
                () -> ((swerve.isOnAllianceSide() && swerve.isFrontFacingAllianceReef()) || (!swerve.isOnAllianceSide() && swerve.isFrontFacingOppositeAllianceReef()))))
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

        // AutonConfig FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA", FourPieceIKLA::new,
        // "Blue I to HP");
        // FOUR_PIECE_IKLA.registerDefault(autonChooser);

        // AutonConfig CHEATER_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA (Cheater)", CheaterFourPieceIKLA::new,
        //  "Blue I to HP");
        //  CHEATER_FOUR_PIECE_IKLA.register(autonChooser);

        AutonConfig PATHFUL_FOUR_PIECE_IKLA = new AutonConfig("4 Piece IKLA (Pathful)", PathfulFourPieceIKLA::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        PATHFUL_FOUR_PIECE_IKLA.register(autonChooser);

        AutonConfig FOUR_PIECE_IKLJ = new AutonConfig("4 Piece IKLJ", FourPieceIKLJ::new,
       "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue J BackOut");
        FOUR_PIECE_IKLJ.register(autonChooser);

        AutonConfig FOUR_PIECE_NUDGE_IKLJ = new AutonConfig("4 Piece Nudge IKLJ", FourPieceNudgeIKLJ::new,
       "Blue Top Nudge", "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue J BackOut");
        FOUR_PIECE_NUDGE_IKLJ.register(autonChooser);

        /** BOTTOM AUTONS **/

        // AutonConfig FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB", FourPieceFDCB::new,
        // "Blue F to HP");
        // FOUR_PIECE_FDCB.register(autonChooser);

        AutonConfig FOUR_PIECE_FDCE = new AutonConfig("4 Piece FDCE", FourPieceFDCE::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue E BackOut");
        FOUR_PIECE_FDCE.register(autonChooser);

        AutonConfig FOUR_PIECE_NUDGE_FDCE = new AutonConfig("4 Piece Nudge FDCE", FourPieceNudgeFDCE::new,
        "Blue Bottom Nudge", "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue E BackOut");
        FOUR_PIECE_NUDGE_FDCE.register(autonChooser);

        // AutonConfig CHEATER_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB (Cheater)", CheaterFourPieceFDCB::new,
        //  "Blue F to HP");
        //  CHEATER_FOUR_PIECE_FDCB.register(autonChooser);

        AutonConfig PATHFUL_FOUR_PIECE_FDCB = new AutonConfig("4 Piece FDCB (Pathful)", PathfulFourPieceFDCB::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        PATHFUL_FOUR_PIECE_FDCB.register(autonChooser);

        /**  TOP ALGAE AUTONS **/

        AutonConfig H_TWO_ALGAE = new AutonConfig("1 Piece H + 2 Algae", OneHTwoAlgae::new,
        "Blue H BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut", "Blue Barge BackOut");
        // AutonConfig H_THREE_ALGAE = new AutonConfig("1 Piece H + 3 Algae (DONT USE)", OneHThreeAlgae::new,
        // "Blue H BackOut", "Blue Barge to IJ (1)", "Blue Barge to EF (1)", "Blue EF BackOut", "Blue Barge BackOut");
        H_TWO_ALGAE.register(autonChooser);
        // H_THREE_ALGAE.register(autonChooser);

        // /** BOTTOM ALGAE AUTONS **/

        AutonConfig G_TWO_ALGAE = new AutonConfig("1 Piece G + 2 Algae", OneGTwoAlgae::new,
        "Blue G BackOut", "Blue Barge to EF (1)", "Blue EF BackOut", "Blue Barge BackOut");
        // AutonConfig G_THREE_ALGAE = new AutonConfig("1 Piece G + 3 Algae (DONT USE)", OneGThreeAlgae::new,
        // "Blue G BackOut", "Blue Barge to IJ (1)", "Blue Barge to EF (1)", "Blue EF BackOut", "Blue Barge BackOut");
        G_TWO_ALGAE.register(autonChooser);
        // G_THREE_ALGAE.register(autonChooser);

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
