package com.stuypulse.robot.commands.arm_elevator.algae;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.commands.arm.algae.*;
import com.stuypulse.robot.commands.elevator.algae.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToAlgaeL3 extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    public MoveToAlgaeL3() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        addRequirements(arm, elevator);

        addCommands(
          new ElevatorToAlgaeL3(),
          new ArmToAlgaeL3()
        );
    }   
}