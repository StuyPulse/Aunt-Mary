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
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climb.ClimbClimb;
import com.stuypulse.robot.commands.climb.ClimbOpen;
import com.stuypulse.robot.commands.climb.ClimbOverrideVoltage;
import com.stuypulse.robot.commands.elevator.ElevatorOffsetTargetDown;
import com.stuypulse.robot.commands.elevator.ElevatorOffsetTargetUp;
import com.stuypulse.robot.commands.elevator.ElevatorOverrideVoltage;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotMoveOperatorOffsetDown;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotMoveOperatorOffsetUp;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        configureDriverButtonBindings();
        configureOperatorButtonBindings();
        configureAutons();
        configureSysids();

        swerve.registerTelemetry(telemetry::telemeterize);
        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULT ***/
    /****************/

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
        driver.getDPadRight().whileTrue(new SwerveDriveDriveAligned(driver, Rotation2d.kZero));

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
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public void configureSysids() {
        autonChooser.setDefaultOption("Swerve Quasi Forward", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.setDefaultOption("Swerve Quasi Backward", swerve.sysIdQuasistatic(Direction.kReverse));
        autonChooser.setDefaultOption("Swerve Dynamic Forward", swerve.sysIdDynamic(Direction.kForward));
        autonChooser.setDefaultOption("Swerve Dynamic Backward", swerve.sysIdDynamic(Direction.kReverse));

        autonChooser.setDefaultOption("Elevator Quasi Forward", elevator.getSysIdQuasistatic(Direction.kForward));
        autonChooser.setDefaultOption("Elevator Quasi Backward", elevator.getSysIdQuasistatic(Direction.kReverse));
        autonChooser.setDefaultOption("Elevator Dynamic Forward", elevator.getSysIdDynamic(Direction.kForward));
        autonChooser.setDefaultOption("Elevator Dynamic Backward", elevator.getSysIdDynamic(Direction.kReverse));

        autonChooser.setDefaultOption("Arm Quasi Forward", arm.getSysIdQuasistatic(Direction.kForward));
        autonChooser.setDefaultOption("Arm Quasi Backward", arm.getSysIdQuasistatic(Direction.kReverse));
        autonChooser.setDefaultOption("Arm Dynamic Forward", arm.getSysIdDynamic(Direction.kForward));
        autonChooser.setDefaultOption("Arm Dynamic Backward", arm.getSysIdDynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
