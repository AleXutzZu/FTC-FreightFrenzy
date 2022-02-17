package org.firstinspires.ftc.teamcode.opmodes.autonomous.demo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;

@Autonomous(name = "Demo: Red Side Far Position", group = "Demo Cluj-Napoca")
public class RedSideFarPosition extends AutonomousControl {

    @Override
    protected void run() {
        driveStraight(25);
        rotate(-90);
        ElevatorLevels level = identifyElement();
        float distance = 50;
        if (level == ElevatorLevels.LEVEL_THREE || level == ElevatorLevels.LEVEL_ONE) distance -= 20;
        useElevator(level);
        driveStraight(distance);
        rotate(90);
        useArm(false);
        distance = 12;
        if (level == ElevatorLevels.LEVEL_THREE) distance += 3;
        if (level == ElevatorLevels.LEVEL_TWO) distance += 2;
        driveStraight(distance);
        useClaws(false);
        driveStraight(-distance);
        rotate(-90);
        driveStraight(150);
        useArm(true);
        useElevator(ElevatorLevels.START);
    }
}
