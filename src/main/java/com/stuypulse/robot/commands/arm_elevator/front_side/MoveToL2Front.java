package com.stuypulse.robot.commands.arm_elevator.front_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.commands.arm.front_side.*;
import com.stuypulse.robot.commands.elevator.front_side.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToL2Front extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    public MoveToL2Front() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        addRequirements(arm, elevator);

        addCommands(
          new ArmToL2Front(),
          new ElevatorToL2Front()
        );
    }   
}