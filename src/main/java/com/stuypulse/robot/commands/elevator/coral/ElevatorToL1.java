package com.stuypulse.robot.commands.elevator.coral;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorToL1 extends ElevatorToHeight {
    public ElevatorToL1() {
        super(ElevatorState.L1);
    }
}
