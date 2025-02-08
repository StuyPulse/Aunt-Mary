package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToFeed extends ElevatorToHeight{
    public ElevatorToFeed() {
        super(Elevator.FUNNEL_HEIGHT_METERS);
    }
}
