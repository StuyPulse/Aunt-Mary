package com.stuypulse.robot.commands.elevator.algae;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorToHoldAlgae extends ElevatorToHeight{
    public ElevatorToHoldAlgae() {
        super(ElevatorState.HOLD_ALGAE);
    }
}
