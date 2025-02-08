package com.stuypulse.robot.commands.auton.routines;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.stuypulse.robot.commands.arm.ArmMoveToAngle;
import com.stuypulse.robot.commands.arm.ArmMoveToFunnel;
import com.stuypulse.robot.commands.arm.ArmMoveToL4Front;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl4Funnel;
import com.stuypulse.robot.commands.lokishooter.ShooterShootFront;
import com.stuypulse.robot.commands.lokishooter.ShooterStop;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.elevator.ElevatorToHandoff;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl3Funnel;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl4Alt;

public class ScoreRoutine extends SequentialCommandGroup {

    public ScoreRoutine() {

        addCommands(
            new ParallelCommandGroup(
                new ArmMoveToL4Front(),
                new ElevatorToLvl4Alt()
            ),

            new ShooterShootFront(), // change to be based on sensor values later
            new WaitCommand(Settings.Auton.SHOOTER_WAIT_TIME),
            new ShooterStop(),

            new ParallelCommandGroup(
                new ArmMoveToFunnel(),
                new ElevatorToHandoff()
            )
        );

    }

}