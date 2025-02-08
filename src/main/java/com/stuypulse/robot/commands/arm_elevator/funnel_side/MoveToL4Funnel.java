package com.stuypulse.robot.commands.arm_elevator.funnel_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.commands.arm.funnel_side.*;
import com.stuypulse.robot.commands.elevator.funnel_side.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToL4Funnel extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    public MoveToL4Funnel() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        addRequirements(arm, elevator);

        addCommands(
          new ElevatorToL4Funnel(),
          new ArmToL4Funnel()
        );
    }
}