package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorToFeed extends ElevatorToHeight{
    public ElevatorToFeed() {
        super(ElevatorState.FEED);
    }
}
