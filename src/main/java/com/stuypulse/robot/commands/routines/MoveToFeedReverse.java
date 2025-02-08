package com.stuypulse.robot.commands.routines;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.elevator.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToFeedReverse extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    public MoveToFeedReverse() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        addRequirements(arm, elevator);

        addCommands(
          new ArmToFeedReverse(),
          new ElevatorToFeed()
        );
    }
}