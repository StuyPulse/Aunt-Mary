package com.stuypulse.robot.commands;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.Clearances;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutine extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private Shooter shooter;

    public ScoreRoutine(int level, boolean isFrontFacingReef) {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();

        ElevatorState elevatorState = Elevator.getState(level, isFrontFacingReef);
        ArmState armState = Arm.getState(level, isFrontFacingReef);

        addCommands(
            new SwerveDriveCoralScoreAlignWithClearance(level, isFrontFacingReef, elevatorState, armState)
                .alongWith(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).alongWith(new ShooterWaitUntilHasCoral())
                    .andThen(RobotContainer.getElevatorArmCommands(level, isFrontFacingReef))
                    .onlyIf(() -> elevator.getState() != elevatorState || arm.getState() != armState)
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // check again since robot may have moved
                .andThen(shooter.shouldShootBackwards() ? new ShooterShootBackwards() : new ShooterShootForwards())
        );
    }
    
}
