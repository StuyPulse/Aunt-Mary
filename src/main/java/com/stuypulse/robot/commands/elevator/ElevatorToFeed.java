package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToFeed extends ElevatorToHeight{
    public ElevatorToFeed() {
        super(Elevator.FEED_HEIGHT_METERS);
    }
}
