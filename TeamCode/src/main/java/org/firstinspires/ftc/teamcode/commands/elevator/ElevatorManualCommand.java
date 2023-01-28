package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions;

public class ElevatorManualCommand extends CommandBase {
    Elevator elevator;
    int targetJunction;
    int allowedError = 20;

    public ElevatorManualCommand(Elevator elevator, int junctions) {
        this.elevator = elevator;
        targetJunction = junctions;
    }

    public ElevatorManualCommand(Elevator elevator, int junctions, int allowedError) {
        this.elevator = elevator;
        targetJunction = junctions;
        this.allowedError = allowedError;
    }

    @Override
    public void initialize() {
        elevator.enableLift();
        elevator.setLevel(targetJunction);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getAbsLiftError()) > allowedError;
    }
}
