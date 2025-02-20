/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import java.util.Optional;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.arm.ArmOffsetTargetDown;
import com.stuypulse.robot.commands.arm.ArmOffsetTargetUp;
import com.stuypulse.robot.commands.arm.ArmOverrideVoltage;
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
import com.stuypulse.robot.commands.autons.misc.OnePieceG;
import com.stuypulse.robot.commands.autons.misc.OnePieceH;
import com.stuypulse.robot.commands.autons.tests.CurvyLineTest;
import com.stuypulse.robot.commands.autons.tests.RSquareTest;
import com.stuypulse.robot.commands.autons.tests.SquareTest;
import com.stuypulse.robot.commands.autons.tests.StraightLineTest;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbOverrideVoltage;
import com.stuypulse.robot.commands.elevator.ElevatorOffsetTargetDown;
import com.stuypulse.robot.commands.elevator.ElevatorOffsetTargetUp;
import com.stuypulse.robot.commands.elevator.ElevatorOverrideVoltage;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotMoveOperatorOffsetDown;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotMoveOperatorOffsetUp;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotOverrideVoltage;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToAlgaeGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToCoralGroundPickup;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToStow;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotWaitUntilAtTargetAngle;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerIntakeCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoral;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerStop;
import com.stuypulse.robot.commands.funnel.FunnelDefaultCommand;
import com.stuypulse.robot.commands.funnel.FunnelReverse;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.leds.LEDSolidColor;
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
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAligned;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedToBarge;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToBarge;
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
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private final Funnel funnel = Funnel.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Arm arm = Arm.getInstance();
    private final SuperStructure superStructure = SuperStructure.getInstance();
    private final Climb climb = Climb.getInstance();
    private final Froggy froggy = Froggy.getInstance();
    private final LEDController leds = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        configureDefaultCommands();
        configureTestButtons();
        // configureDriverButtonBindings();
        // configureOperatorButtonBindings();
        configureAutons();
        configureSysids();

        swerve.registerTelemetry(telemetry::telemeterize);
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULT ***/
    /****************/

    private void configureTestButtons() {
        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        driver.getTopButton().onTrue(new SuperStructureToL4Front());
        driver.getRightButton().onTrue(new SuperStructureToL3Front());
        driver.getBottomButton().onTrue(new SuperStructureToFeed());

        driver.getRightTriggerButton().whileTrue(new FroggyRollerIntakeCoral());

        driver.getDPadRight().whileTrue(new ShooterShootForwards()).onFalse(new ShooterStop());
        driver.getDPadLeft().whileTrue(new ShooterShootBackwards()).onFalse(new ShooterStop());
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        funnel.setDefaultCommand(new FunnelDefaultCommand());
        leds.setDefaultCommand(new LEDDefaultCommand());
        shooter.setDefaultCommand(new ShooterAcquireCoral().andThen(new BuzzController(driver)).onlyIf(() -> !shooter.hasCoral()));
    }

    /***************/
    /*** BUTTON ***/
    /***************/

    private void configureDriverButtonBindings() {

        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        // manual shoot depending on whatever states robot is in: either score barge, forwards/backwards on reef, or processor/L1
        driver.getDPadRight()
            .whileTrue(new ConditionalCommand(
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
                () -> superStructure.isInScoreState()))
            .onFalse(new ShooterStop())
            .onFalse(new FroggyRollerStop());

        // ground algae pickup
        driver.getLeftTriggerButton()
            .onTrue(new FroggyPivotToAlgaeGroundPickup())
            .whileTrue(new FroggyRollerIntakeAlgae().andThen(new BuzzController(driver)))
            .onFalse(new FroggyPivotToStow());

        driver.getLeftBumper()
            .onTrue(new FroggyRollerShootAlgae())
            .onFalse(new FroggyRollerStop());

        driver.getRightTriggerButton()
            .onTrue(new FroggyPivotToCoralGroundPickup())
            .whileTrue(new FroggyRollerIntakeCoral())
            .onFalse(new FroggyPivotToStow());

        driver.getRightBumper()
            .whileTrue(new FroggyPivotToL1()
                .andThen(new FroggyPivotWaitUntilAtTargetAngle())
                .andThen(new FroggyRollerShootCoral()))
            .onFalse(new FroggyRollerStop().andThen(new FroggyPivotToStow()));

        driver.getTopButton()
            .whileTrue(
                new ConditionalCommand(
                    new SuperStructureToL4Front()
                        .andThen(new SuperStructureWaitUntilAtTarget()
                            .alongWith(new SwerveDrivePIDToNearestBranch(4, true)
                                .alongWith(new LEDSolidColor(Color.kYellow))))
                        .andThen(new ShooterShootBackwards()), 
                    new SuperStructureToL4Back()
                        .andThen(new SuperStructureWaitUntilAtTarget()
                            .alongWith(new SwerveDrivePIDToNearestBranch(4, false)
                                .alongWith(new LEDSolidColor(Color.kYellow))))
                        .andThen(new ShooterShootForwards()), 
                    () -> swerve.isFrontFacingReef())
            )
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getRightButton()
            .whileTrue(
                new ConditionalCommand(
                    new SuperStructureToL3Front()
                        .andThen(new SuperStructureWaitUntilAtTarget()
                            .alongWith(new SwerveDrivePIDToNearestBranch(3, true)
                                .alongWith(new LEDSolidColor(Color.kYellow))))
                        .andThen(new ShooterShootBackwards()), 
                    new SuperStructureToL3Back()
                        .andThen(new SuperStructureWaitUntilAtTarget()
                            .alongWith(new SwerveDrivePIDToNearestBranch(3, false)
                                .alongWith(new LEDSolidColor(Color.kYellow))))
                        .andThen(new ShooterShootForwards()), 
                    () -> swerve.isFrontFacingReef())
            )
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getBottomButton()
            .whileTrue(
                new ConditionalCommand(
                    new SuperStructureToL2Front()
                        .andThen(new SuperStructureWaitUntilAtTarget()
                            .alongWith(new SwerveDrivePIDToNearestBranch(2, true)
                                .alongWith(new LEDSolidColor(Color.kYellow))))
                        .andThen(new ShooterShootForwards()), 
                    new SuperStructureToL2Back()
                        .andThen(new SuperStructureWaitUntilAtTarget()
                            .alongWith(new SwerveDrivePIDToNearestBranch(2, false)
                                .alongWith(new LEDSolidColor(Color.kYellow))))
                        .andThen(new ShooterShootForwards()), 
                    () -> swerve.isFrontFacingReef())
            )
            .onFalse(new SuperStructureToFeed())
            .onFalse(new ShooterStop());

        driver.getLeftButton()
            .whileTrue(new SwerveDriveDriveAlignedToBarge(driver))
            .whileTrue(new SuperStructureToBarge()
                .andThen(new SuperStructureWaitUntilAtTarget()
                    .alongWith(new SwerveDriveWaitUntilAlignedToBarge()
                        .alongWith(new LEDSolidColor(Color.kYellow))))
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

    private void configureOperatorButtonBindings() {
        new Trigger(() -> Math.abs(operator.getLeftStick().y) > Settings.Operator.Elevator.VOLTAGE_OVERRIDE_DEADBAND)   
            .whileTrue(new ElevatorOverrideVoltage(() -> -operator.getLeftStick().y > 0 
                ? (-operator.getLeftStick().y * Math.abs(Settings.Operator.Elevator.MAX_VOLTAGE_UP))
                : (-operator.getLeftStick().y * Math.abs(Settings.Operator.Elevator.MAX_VOLTAGE_DOWN))));
        
        new Trigger(() -> Math.abs(operator.getRightStick().x) > Settings.Operator.Arm.VOLTAGE_OVERRIDE_DEADBAND)
            .whileTrue(new ArmOverrideVoltage(() -> operator.getRightStick().x > 0 
                ? (operator.getRightStick().x * Math.abs(Settings.Operator.Arm.MAX_VOLTAGE_UP))
                : (operator.getRightStick().x * Math.abs(Settings.Operator.Arm.MAX_VOLTAGE_DOWN))));

        operator.getLeftTriggerButton()
            .whileTrue(new ConditionalCommand(
                new ShooterShootForwards().alongWith(new FunnelReverse()),
                new ShooterShootBackwards().alongWith(new FunnelReverse()),
                () -> shooter.shouldShootBackwards()))
            .onFalse(new ShooterStop());

        operator.getLeftBumper().whileTrue(new FroggyPivotMoveOperatorOffsetUp());
        operator.getRightBumper().whileTrue(new FroggyPivotMoveOperatorOffsetDown());

        operator.getLeftMenuButton().onTrue(new ClimbOverrideVoltage(Settings.Operator.Climb.CLIMB_DOWN_VOLTAGE));
        operator.getRightMenuButton().onTrue(new ClimbOverrideVoltage(Settings.Operator.Climb.CLIMB_UP_VOLTAGE)); 

        operator.getTopButton().onTrue(swerve.isFrontFacingReef() ? new SuperStructureToL4Front() : new SuperStructureToL4Back());
        operator.getRightButton().onTrue(swerve.isFrontFacingReef() ? new SuperStructureToL3Front() : new SuperStructureToL3Back());
        operator.getBottomButton().onTrue(swerve.isFrontFacingReef() ? new SuperStructureToL2Front() : new SuperStructureToL2Back());
        operator.getLeftButton().onTrue(new SuperStructureToBarge());

        operator.getDPadUp().onTrue(new ElevatorOffsetTargetUp());
        operator.getDPadDown().onTrue(new ElevatorOffsetTargetDown());

        operator.getDPadLeft().onTrue(new ArmOffsetTargetDown());
        operator.getDPadRight().onTrue(new ArmOffsetTargetUp());

        operator.getRightTriggerButton()
            .whileTrue(new ConditionalCommand(
                new ShooterShootBackwards(), 
                new ShooterShootForwards(), 
                () -> shooter.shouldShootBackwards()));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        swerve.configureAutoBuilder();

        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        /** TOP AUTONS **/

        AutonConfig BLUE_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Blue Mid Top to H");
        AutonConfig RED_ONE_PIECE_H = new AutonConfig("1 Piece H", OnePieceH::new,
        "Red Mid Top to H");

        AutonConfig BLUE_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePieceJ::new,
        "Blue Top to J");
        AutonConfig RED_ONE_PIECE_J = new AutonConfig("1 Piece J", OnePieceJ::new,
        "Red Top to J");

        AutonConfig BLUE_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPieceJK::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K");
        AutonConfig RED_TWO_PIECE_JK = new AutonConfig("2 Piece JK", TwoPieceJK::new,
        "Red Top to J", "Red J to HP", "Red HP to K");

        AutonConfig BLUE_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePieceJKL::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L");
        AutonConfig RED_THREE_PIECE_JKL = new AutonConfig("3 Piece JKL", ThreePieceJKL::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L");

        AutonConfig BLUE_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPieceJKL::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP");
        AutonConfig RED_THREE_HALF_PIECE_JKL = new AutonConfig("3.5 Piece JKL", ThreeHalfPieceJKL::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Blue L to HP");

        AutonConfig BLUE_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPieceJKLA::new,
        "Blue Top to J", "Blue J to HP", "Blue HP to K", "Blue K to HP", "Blue HP to L", "Blue L to HP","Red HP to A");
        AutonConfig RED_FOUR_PIECE_JKLA = new AutonConfig("4 Piece JKLA", FourPieceJKLA::new,
        "Red Top to J", "Red J to HP", "Red HP to K", "Red K to HP", "Red HP to L", "Blue L to HP", "Red HP to A");
        
        BLUE_ONE_PIECE_H.registerBlue(autonChooser);
        RED_ONE_PIECE_H.registerRed(autonChooser);
       
        BLUE_ONE_PIECE_J.registerBlue(autonChooser);
        RED_ONE_PIECE_J.registerRed(autonChooser);

        BLUE_TWO_PIECE_JK.registerBlue(autonChooser);
        RED_TWO_PIECE_JK.registerRed(autonChooser);

        BLUE_THREE_PIECE_JKL.registerBlue(autonChooser);
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
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Blue C to HP");

        AutonConfig BLUE_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPieceEDCB::new,
        "Blue Bottom to E", "Blue E to HP", "Blue HP to D", "Blue D to HP", "Blue HP to C", "Blue C to HP","Red HP to B");
        AutonConfig RED_FOUR_PIECE_EDCB = new AutonConfig("4 Piece EDCB", FourPieceEDCB::new,
        "Red Bottom to E", "Red E to HP", "Red HP to D", "Red D to HP", "Red HP to C", "Blue C to HP", "Red HP to B");

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

        BLUE_MOBILITY.registerBlue(autonChooser);
        RED_MOBILITY.registerRed(autonChooser);
        STRAIGHT_LINE_TEST.registerRed(autonChooser);
        CURVY_LINE_TEST.registerRed(autonChooser);
        SQUARE_TEST.registerRed(autonChooser);
        RSQUARE_TEST.registerRed(autonChooser);

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
