
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.ManualShoot;
import com.stuypulse.robot.commands.ReefAlgaePickupRoutineBack;
import com.stuypulse.robot.commands.ReefAlgaePickupRoutineFront;
import com.stuypulse.robot.commands.Reset;
import com.stuypulse.robot.commands.ScoreRoutine;
import com.stuypulse.robot.commands.autons.FDCB.FDCB;
import com.stuypulse.robot.commands.autons.FDCB.FDCBL2;
import com.stuypulse.robot.commands.autons.FDCB.FDCBNudge;
import com.stuypulse.robot.commands.autons.FDCB.FDCE;
import com.stuypulse.robot.commands.autons.FDCB.FDCENudge;
import com.stuypulse.robot.commands.autons.HAlgae.HTwoAlgae;
import com.stuypulse.robot.commands.autons.IKLA.IKLA;
import com.stuypulse.robot.commands.autons.IKLA.IKLAL2;
import com.stuypulse.robot.commands.autons.IKLA.IKLANudge;
import com.stuypulse.robot.commands.autons.IKLA.IKLJ;
import com.stuypulse.robot.commands.autons.IKLA.IKLJNudge;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbIdle;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbShimmy;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToCoralGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1One;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1Three;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1Two;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1Versatile;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToStow;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilCanMoveWithoutColliding;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerHoldAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerHoldCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoralOne;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoralThree;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoralTwo;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoralVersatile;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerStop;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterUnjamCoralBackwards;
import com.stuypulse.robot.commands.shooter.scoring.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.scoring.ShooterShootL1Front;
import com.stuypulse.robot.commands.superStructure.SuperStructureClimb;
import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeSafe118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureBarge118;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureGolfTeeAlgaePickup;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureGroundAlgaePickup;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureProcessor;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL2Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL3Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL3Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapult;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBarge118Clearance;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedToBarge118Score;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestL1ShooterReady;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDAssistToClosestL1ShooterScore;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyReady;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyScore;
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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    // Gamepads
    public final CommandXboxController driver = new CommandXboxController(Ports.Gamepad.DRIVER);
    public final CommandXboxController operator = new CommandXboxController(Ports.Gamepad.OPERATOR);

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
        // testingButtonBindings();
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

    // private void testingButtonBindings(){
    //     driver.getTopButton().onTrue(new SuperStructureCoralL4Back()).onFalse(new SuperStructureFeed());
    //     driver.getLeftButton().onTrue(new SuperStructureCoralL4Front()).onFalse(new SuperStructureFeed());
    // }

    private void configureDriverButtonBindings() {

        driver.povUp().onTrue(new SwerveDriveResetRotation());

        // Manual Shoot
        driver.povRight()
            .onTrue(
                new ConditionalCommand(
                    new ConditionalCommand(
                        new ConditionalCommand(
                        new FroggyRollerShootCoralVersatile(), 
                        new ConditionalCommand(
                            new FroggyRollerShootCoralOne(), 
                            new ConditionalCommand(
                                new FroggyRollerShootCoralTwo(),
                                new FroggyRollerShootCoralThree(),
                                () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_TWO
                            ), 
                            () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_ONE), 
                        () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_VERSATILE),
                    new ManualShoot(),
                    () -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_VERSATILE ||  froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_ONE ||  froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_TWO ||  froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_THREE
                ),
                new ShooterShootAlgae().andThen(new WaitCommand(0.2).andThen(new SuperStructureAlgaeSafe118()).onlyIf(() -> superStructure.getState() == SuperStructureState.BARGE_118)),
                () -> shooter.getState() != ShooterState.HOLD_ALGAE && superStructure.getState() != SuperStructureState.BARGE_SAFE)
                )
            .whileTrue(new LEDApplyPattern(Settings.LED.MANUAL_SHOOT_COLOR))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isArmClearFromBarge())
                .andThen(new SuperStructureFeed().onlyIf(() -> superStructure.getState() == SuperStructureState.PROCESSOR || superStructure.getState() == SuperStructureState.BARGE_SAFE || superStructure.isScoringCoral() || shooter.getState() != ShooterState.HOLD_ALGAE)))
            .onFalse(new FroggyRollerStop()
                .onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL && froggy.getRollerState() != RollerState.HOLD_ALGAE))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow()));

        // ground froggy algae intake and reset
        driver.leftTrigger()
            .onTrue(new Reset())
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .onTrue(new FroggyRollerIntakeAlgae())
            .onFalse(new FroggyPivotToStow())
            .onFalse(new FroggyRollerHoldAlgae());

        // Loki golf tee algae pickup
        driver.leftBumper()
            .onTrue(new SuperStructureGolfTeeAlgaePickup())
            .onTrue(new ShooterAcquireAlgae())
            .onFalse(new SuperStructureProcessor())                                
            .onFalse(new ShooterHoldAlgae());

        // Ground coral intake and send elevator/arm to feed
        driver.rightTrigger()
            .onTrue(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.CORAL_GROUND_PICKUP)
                .andThen(new FroggyPivotToCoralGroundPickup().alongWith(new FroggyRollerIntakeCoral())))
            .onFalse(new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.STOW)
                .andThen(new FroggyPivotToStow()))
            .onFalse(new FroggyRollerHoldCoral()); 

        // L1
        driver.rightBumper()
            .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE_VERSATILE) && !shooter.hasCoral()))
            .whileTrue(new ConditionalCommand(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new ConditionalCommand(
                            new SuperStructureCoralL1Front(),
                            new SuperStructureCoralL1Back(),
                            () -> swerve.isFrontFacingAllianceReef())),
                new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE_VERSATILE)
                    .andThen(new FroggyPivotToL1Versatile()), 
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow().alongWith(new FroggyRollerStop()))
                .onlyIf(() -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_VERSATILE && (froggy.getRollerState() == RollerState.SHOOT_CORAL_VERSATILE || froggy.getRollerState() == RollerState.STOP)))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_CORAL_L1_FRONT || shooter.getState() == ShooterState.SHOOT_CORAL_L1_BACK));
        
        driver.rightBumper().debounce(0.25)
            .whileTrue(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR)
                .until(() -> shooter.getState() == ShooterState.SHOOT_CORAL_L1_FRONT || 
                             shooter.getState() == ShooterState.SHOOT_CORAL_L1_FRONT || 
                             froggy.getRollerState() == RollerState.SHOOT_CORAL_VERSATILE))
            .whileTrue(new ConditionalCommand(
                // new WaitUntilCommand(() -> superStrucutre.getState() == SuperStructureState.L1)
                new WaitUntilCommand(() -> superStructure.getState() == SuperStructureState.L1_FRONT && superStructure.atTarget())
                    .deadlineFor(new SwerveDrivePIDAssistToClosestL1ShooterReady(driver))
                    .andThen(new SwerveDrivePIDAssistToClosestL1ShooterScore(driver)
                        .alongWith(new WaitUntilCommand(() -> ReefUtil.getClosestReefFace().isAlignedToL1ShooterTarget())).andThen(new ShooterShootL1Front()))
                        .andThen(
                new ConditionalCommand(
                    new ScoreRoutine(driver, 1, true).until(() -> false),
                    new ScoreRoutine(driver, 1, false).until(() -> false),
                    () -> swerve.isFrontFacingAllianceReef())
                ),
                new ConditionalCommand(
                    new FroggyPivotToL1Versatile()
                    .deadlineFor(new LEDApplyPattern(Settings.LED.SCORE_COLOR))
                    .alongWith(
                            new SwerveDrivePIDToClosestL1FroggyScore(0)
                                .andThen(new FroggyRollerShootCoralVersatile())), 
                new SwerveDrivePIDToClosestL1FroggyReady().alongWith(new FroggyPivotToL1Versatile())
                    .alongWith(
                new WaitUntilCommand(() -> froggy.getCurrentAngle().getDegrees() > PivotState.L1_SCORE_ANGLE_VERSATILE.getTargetAngle().getDegrees() - 10))
                        .andThen(
                            new SwerveDrivePIDToClosestL1FroggyScore(0)
                                .andThen(new FroggyRollerShootCoralVersatile())),
                () -> Clearances.isFroggyClearFromAllObstables()),
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new SuperStructureFeed()).onlyIf(() -> superStructure.getState() == SuperStructureState.L1_FRONT || superStructure.getState() == SuperStructureState.L1_BACK || shooter.getState() != ShooterState.HOLD_ALGAE))
            .onFalse(new FroggyRollerStop().onlyIf(() -> froggy.getRollerState() != RollerState.HOLD_CORAL));

        // L4 Coral Score + Top L1
        driver.y()
        .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE_THREE) && !shooter.hasCoral()))
            .whileTrue(new ConditionalCommand(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new ConditionalCommand(
                            new SuperStructureCoralL4Front(),
                            new SuperStructureCoralL4Back(),
                            () -> swerve.isFrontFacingAllianceReef())),
                            new ConditionalCommand(
                        new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE_THREE)
                            .andThen(new FroggyPivotToL1Three()),
                        new FroggyPivotToStow(), 
                        () -> froggy.getRollerState() == RollerState.HOLD_CORAL), 
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow().alongWith(new FroggyRollerStop()))
                .onlyIf(() -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_THREE && (froggy.getRollerState() == RollerState.SHOOT_CORAL_THREE || froggy.getRollerState() == RollerState.STOP)))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.isShootingCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isFroggyClearFromAllObstables())
                .andThen(new SuperStructureFeed().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE)));

        driver.y().debounce(0.25)
        .whileTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    new FroggyPivotToL1Three()
                    .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_THREE))
                    .alongWith(
                            new SwerveDrivePIDToClosestL1FroggyScore(3)
                                .andThen(new FroggyRollerShootCoralThree())),
                new SwerveDrivePIDToClosestL1FroggyReady().alongWith(new FroggyPivotToL1Three())
                    .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_THREE))
                    .alongWith(
                new WaitUntilCommand(() -> froggy.getCurrentAngle().getDegrees() > PivotState.L1_SCORE_ANGLE_THREE.getTargetAngle().getDegrees() - 10))
                        .andThen(
                            new SwerveDrivePIDToClosestL1FroggyScore(3)
                                .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_THREE))
                                .andThen(new FroggyRollerShootCoralThree())),
                () -> Clearances.isFroggyClearFromAllObstables()),
                    new ConditionalCommand(
                        new ScoreRoutine(driver, 4, true).until(() -> false),
                        new ScoreRoutine(driver, 4, false).until(() -> false), 
                        () -> swerve.isFrontFacingAllianceReef()), 
            () -> !shooter.hasCoral() && froggy.getRollerState() == RollerState.HOLD_CORAL)
        )
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isFroggyClearFromAllObstables())
            .andThen(new SuperStructureFeed().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE)).alongWith(new FroggyPivotToStow()).andThen(new FroggyRollerStop()))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.isShootingCoral()));

        // L3 Coral Score + 2nd L1
        driver.b()
        .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE_TWO) && !shooter.hasCoral()))
            .whileTrue(new ConditionalCommand(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new ConditionalCommand(
                            new SuperStructureCoralL3Front(),
                            new SuperStructureCoralL3Back(),
                            () -> swerve.isFrontFacingAllianceReef())),
                new ConditionalCommand(
                    new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE_TWO)
                        .andThen(new FroggyPivotToL1Two()),
                    new FroggyPivotToStow(), 
                    () -> froggy.getRollerState() == RollerState.HOLD_CORAL), 
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow().alongWith(new FroggyRollerStop()))
                .onlyIf(() -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_TWO && (froggy.getRollerState() == RollerState.SHOOT_CORAL_TWO || froggy.getRollerState() == RollerState.STOP)))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.isShootingCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isFroggyClearFromAllObstables())
                .andThen(new SuperStructureFeed().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE)));
        
            driver.b().debounce(0.25)
            .whileTrue(
                    new ConditionalCommand(
                        new ConditionalCommand(
                            new FroggyPivotToL1Two()
                            .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_TWO))
                            .alongWith(
                                    new SwerveDrivePIDToClosestL1FroggyScore(2)
                                        .andThen(new FroggyRollerShootCoralTwo())),
                        new SwerveDrivePIDToClosestL1FroggyReady().alongWith(new FroggyPivotToL1Two())
                            .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_TWO))
                            .alongWith(
                        new WaitUntilCommand(() -> froggy.getCurrentAngle().getDegrees() > PivotState.L1_SCORE_ANGLE_TWO.getTargetAngle().getDegrees() - 10))
                                .andThen(
                                    new SwerveDrivePIDToClosestL1FroggyScore(2)
                                        .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_TWO))
                                        .andThen(new FroggyRollerShootCoralTwo())),
                        () -> Clearances.isFroggyClearFromAllObstables()),
                            new ConditionalCommand(
                                new ScoreRoutine(driver, 3, true).until(() -> false),
                                new ScoreRoutine(driver, 3, false).until(() -> false), 
                                () -> swerve.isFrontFacingAllianceReef()), 
                    () -> !shooter.hasCoral() && froggy.getRollerState() == RollerState.HOLD_CORAL)
                )
        .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isFroggyClearFromAllObstables())
                .andThen(new SuperStructureFeed().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE)).alongWith(new FroggyPivotToStow()).andThen(new FroggyRollerStop()))
        .onFalse(new ShooterStop().onlyIf(() -> shooter.isShootingCoral()));

        // L2 Coral Score + Bottom L1
        driver.a()
        .onTrue(new BuzzController(driver).onlyIf(() -> !Clearances.canMoveFroggyWithoutColliding(PivotState.L1_SCORE_ANGLE_ONE) && !shooter.hasCoral()))
            .whileTrue(new ConditionalCommand(
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new ConditionalCommand(
                            new SuperStructureCoralL2Front(),
                            new SuperStructureCoralL2Front(),
                            () -> swerve.isFrontFacingAllianceReef())),
                        new ConditionalCommand(
                            new FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState.L1_SCORE_ANGLE_ONE)
                                .andThen(new FroggyPivotToL1One()),
                            new FroggyPivotToStow(), 
                            () -> froggy.getRollerState() == RollerState.HOLD_CORAL), 
                () -> shooter.hasCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isFroggyClearFromAllObstables())
                .andThen(new FroggyPivotToStow().alongWith(new FroggyRollerStop()))
                .onlyIf(() -> froggy.getPivotState() == PivotState.L1_SCORE_ANGLE_ONE && (froggy.getRollerState() == RollerState.SHOOT_CORAL_ONE || froggy.getRollerState() == RollerState.STOP)))
            .onFalse(new ShooterStop().onlyIf(() -> shooter.isShootingCoral()))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isFroggyClearFromAllObstables())
                .andThen(new SuperStructureFeed().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE)));

        driver.a().debounce(0.25)
        .whileTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    new FroggyPivotToL1One()
                    .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_ONE))
                    .alongWith(
                            new SwerveDrivePIDToClosestL1FroggyScore(1)
                                .andThen(new FroggyRollerShootCoralOne())),
                new SwerveDrivePIDToClosestL1FroggyReady().alongWith(new FroggyPivotToL1One())
                    .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_ONE))
                    .alongWith(
                new WaitUntilCommand(() -> froggy.getCurrentAngle().getDegrees() > PivotState.L1_SCORE_ANGLE_ONE.getTargetAngle().getDegrees() - 10))
                        .andThen(
                            new SwerveDrivePIDToClosestL1FroggyScore(1)
                                .deadlineFor(new LEDApplyPattern(Settings.LED.FROGGY_SCORE_ONE))
                                .andThen(new FroggyRollerShootCoralOne())),
                () -> Clearances.isFroggyClearFromAllObstables()),
                    new ConditionalCommand(
                        new ScoreRoutine(driver, 2, true).until(() -> false),
                        new ScoreRoutine(driver, 2, false).until(() -> false), 
                        () -> swerve.isFrontFacingAllianceReef()), 
            () -> !shooter.hasCoral() && froggy.getRollerState() == RollerState.HOLD_CORAL)
        )
        .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef() && Clearances.isFroggyClearFromAllObstables())
                .andThen(new SuperStructureFeed().onlyIf(() -> shooter.getState() != ShooterState.HOLD_ALGAE)).alongWith(new FroggyPivotToStow()).andThen(new FroggyRollerStop()))
        .onFalse(new ShooterStop().onlyIf(() -> shooter.isShootingCoral()));
        
        // 118 Auto Score
        driver.x()
            .whileTrue(
                new SwerveDriveDriveAlignedToBarge118Clearance(driver, false)
                    .deadlineFor(new LEDApplyPattern(Settings.LED.BARGE_ALIGNING))
                    .until(() -> superStructure.getState() == SuperStructureState.BARGE_118 && superStructure.canSkipClearance())
                    .andThen(new SwerveDriveDriveAlignedToBarge118Score(driver, false))
                    .alongWith(new WaitUntilCommand(() -> Clearances.isArmClearFromBarge() && Clearances.isArmClearFromReef())
                        .andThen(new SuperStructureBarge118()
                            .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDriveWaitUntilAlignedToCatapult())))
                                .andThen(new WaitCommand(0.3)
                                    .andThen(new ManualShoot()
                                        .alongWith(new WaitUntilCommand(() -> shooter.getState() != ShooterState.HOLD_ALGAE)
                                            .andThen(new WaitCommand(0.1)))
                                            .andThen(new SuperStructureAlgaeSafe118()))))
            )
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromBarge())
                .andThen(new SuperStructureFeed()))
            .onFalse(new WaitUntilCommand(() -> shooter.getState() == ShooterState.SHOOT_ALGAE)
                .andThen(new WaitCommand(0.2)).andThen(new ShooterAcquireAlgae()));
            // .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));
        
        // Align to closest Coral Station
        // driver.getRightStickButton()
        //     .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
        //     .onTrue(SwerveDriveDynamicObstacles.reefClearance())
        //     .onTrue(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new Reset()).onlyIf(() -> !shooter.hasCoral()))
        //     .whileTrue(SwerveDrivePathFindToPose.pathFindToNearestCoralStation()
        //         .until(() -> swerve.getPose().getX() < Field.ALLIANCE_REEF_CENTER.getX())
        //         .andThen(new SwerveDrivePIDAssistToClosestCoralStation(driver))
        //         .alongWith(new LEDApplyPattern(Settings.LED.CORAL_STATION_ALIGN_COLOR))
        //         .onlyIf(() -> !shooter.hasCoral()))
        //     .onFalse(SwerveDriveDynamicObstacles.reset());

        // Align to closest Coral Station without path finding
        // driver.getRightStickButton()
        //     .onTrue(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new Reset()).onlyIf(() -> !shooter.hasCoral()))
        //     .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
        //     .whileTrue(new SwerveDrivePIDToCoralStation(driver)
        //         .onlyIf(() -> !shooter.hasCoral()));

        // Acquire closest reef algae
        driver.povLeft()
            .whileTrue(new ConditionalCommand(
                new ReefAlgaePickupRoutineFront(),
                new ReefAlgaePickupRoutineBack(),
                () -> ((swerve.isOnAllianceSide() && swerve.isFrontFacingAllianceReef()) || (!swerve.isOnAllianceSide() && swerve.isFrontFacingOppositeAllianceReef()))))
            .whileTrue(new LEDApplyPattern(Settings.LED.INTAKE_COLOR_ALGAE))
            .onFalse(new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                .andThen(new SuperStructureProcessor()))
            .onFalse(new ShooterHoldAlgae());

        // Golf tee and Climb Shimmy
        driver.povDown()
            .onTrue(new ConditionalCommand(
                new ClimbShimmy(),
                new SuperStructureGroundAlgaePickup().alongWith(new ShooterAcquireAlgae()),
                () -> climb.getState() != ClimbState.CLOSED
            ))
            .onFalse(
                new ConditionalCommand(
                    new SuperStructureClimb(), 
                    new SuperStructureProcessor().alongWith(new ShooterHoldAlgae()), 
                () -> climb.getState() != ClimbState.CLOSED));

        // Get ready for climb
        driver.back()
            .onTrue(new FroggyPivotToStow())
            .onTrue(new SuperStructureClimb()
                .andThen(new WaitUntilCommand(() -> Elevator.getInstance().atTargetHeight() || Arm.getInstance().atTargetAngle()))
                .andThen(new ClimbOpen()
                    .alongWith(new ShooterStop())
                    .alongWith(new FroggyRollerStop())));

        // // Climb!!
        driver.start()
            .onTrue(new ClimbClimb()
                .onlyIf(() -> climb.getState() == ClimbState.OPEN 
                    || climb.getState() == ClimbState.SHIMMY 
                    || climb.getState() == ClimbState.IDLE))
            .onTrue(new ShooterUnjamCoralBackwards().onlyIf(() -> climb.getState() == ClimbState.CLOSED))
            .onFalse(new ClimbIdle().onlyIf(() -> climb.getState() == ClimbState.CLIMBING))
            .onFalse(new ShooterStop());

        // (UNUSED) Catapult
        // driver.getLeftButton()
        //     .whileTrue(new SwerveDriveDriveAlignedToCatapult(driver)
        //         .deadlineFor(new LEDApplyPattern(Settings.LED.DEFAULT_ALIGN_COLOR))
        //         .alongWith(new SuperStructureCatapultReady()
        //             .andThen(new SuperStructureWaitUntilAtTarget()
        //                 .alongWith(new SwerveDriveWaitUntilAlignedToCatapult()))
        //             .andThen(new SuperStructureCatapultShoot()
        //                 .andThen(new SuperStructureWaitUntilCanCatapult()
        //                     .andThen(new ShooterShootAlgae())))))
        //     .onFalse(new SuperStructureFeed())
        //     .onFalse(new ShooterStop().onlyIf(() -> shooter.getState() == ShooterState.SHOOT_ALGAE));
        
        // Align to closest Coral Station
        // driver.getRightStickButton()
        //     .onTrue(new BuzzController(driver).onlyIf(() -> shooter.hasCoral()))
        //     .onTrue(SwerveDriveDynamicObstacles.reefClearance())
        //     .onTrue(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new Reset()).onlyIf(() -> !shooter.hasCoral()))
        //     .whileTrue(SwerveDrivePathFindToPose.pathFindToNearestCoralStation()
        //         .until(() -> swerve.getPose().getX() < Field.ALLIANCE_REEF_CENTER.getX())
        //         .andThen(new SwerveDrivePIDAssistToClosestCoralStation(driver))
        //         .alongWith(new LEDApplyPattern(Settings.LED.CORAL_STATION_ALIGN_COLOR))
        //         .onlyIf(() -> !shooter.hasCoral()))
        //     .onFalse(SwerveDriveDynamicObstacles.reset());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        autonChooser.addOption("Do Nothing", new DoNothingCommand());

        /** TOP AUTONS **/

        AutonConfig IKLA_AUTON = new AutonConfig("IKLA", IKLA::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        IKLA_AUTON.register(autonChooser);

        AutonConfig IKLA_L2_AUTON = new AutonConfig("IKLA L2 LAST PIECE", IKLAL2::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        IKLA_L2_AUTON.register(autonChooser);

        AutonConfig IKLJ_AUTON = new AutonConfig("IKLJ", IKLJ::new,
        "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue J BackOut");
        IKLJ_AUTON.register(autonChooser);

        AutonConfig IKLJ_NUDGE_AUTON = new AutonConfig("IKLJ Nudge", IKLJNudge::new,
        "Blue Top Nudge", "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue J BackOut");
        IKLJ_NUDGE_AUTON.register(autonChooser);

        AutonConfig IKLA_NUDGE_AUTON = new AutonConfig("IKLA Nudge", IKLANudge::new,
        "Blue Top Nudge", "Blue I to HP", "Blue K to HP", "Blue L to HP", "Blue A BackOut");
        IKLA_NUDGE_AUTON.register(autonChooser);

        /** BOTTOM AUTONS **/

        AutonConfig FDCE_AUTON = new AutonConfig("FDCE", FDCE::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue E BackOut");
        FDCE_AUTON.register(autonChooser);

        AutonConfig FDCE_NUDGE_AUTON = new AutonConfig("FDCE Nudge", FDCENudge::new,
        "Blue Bottom Nudge", "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue E BackOut");
        FDCE_NUDGE_AUTON.register(autonChooser);

        AutonConfig FDCB_AUTON = new AutonConfig("FDCB", FDCB::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        FDCB_AUTON.registerDefault(autonChooser);

        AutonConfig FDCB_L2_AUTON = new AutonConfig("FDCB L2 LAST PIECE", FDCBL2::new,
        "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        FDCB_L2_AUTON.register(autonChooser);

        AutonConfig FDCB_NUDGE_AUTON = new AutonConfig("FDCB Nudge", FDCBNudge::new,
        "Blue Bottom Nudge", "Blue F to HP", "Blue D to HP", "Blue C to HP", "Blue B BackOut");
        FDCB_NUDGE_AUTON.register(autonChooser);

        /**  TOP ALGAE AUTONS **/

        AutonConfig H_TWO_ALGAE_AUTON = new AutonConfig("H + 2 Algae", HTwoAlgae::new,
        "Blue H BackOut", "Blue Barge to IJ (1)", "Blue IJ BackOut", "Blue Barge BackOut");
        H_TWO_ALGAE_AUTON.register(autonChooser);

        // /** BOTTOM ALGAE AUTONS **/

        // AutonConfig G_TWO_ALGAE_AUTON = new AutonConfig("G + 2 Algae", GTwoAlgae::new,
        // "Blue G BackOut", "Blue Barge to EF (1)", "Blue EF BackOut", "Blue Barge BackOut");
        // G_TWO_ALGAE_AUTON.register(autonChooser);

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