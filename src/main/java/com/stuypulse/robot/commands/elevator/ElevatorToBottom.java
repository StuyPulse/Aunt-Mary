package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Constants.Elevator;

public class ElevatorToBottom extends ElevatorToHeight{
    public ElevatorToBottom(){
        super(Elevator.MIN_HEIGHT_METERS);
    }

    public void initialize() {
        super.initialize();
    }
}
