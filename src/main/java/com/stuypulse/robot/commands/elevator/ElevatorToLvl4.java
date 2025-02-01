package com.stuypulse.robot.commands.elevator;
import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl4 extends ElevatorToHeight{
    public ElevatorToLvl4() {
        super(Elevator.L4_HEIGHT_METERS);
    }

    public void initialize() {
        super.initialize();
    }
}
