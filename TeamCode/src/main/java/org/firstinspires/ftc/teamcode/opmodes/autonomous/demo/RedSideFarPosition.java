package org.firstinspires.ftc.teamcode.opmodes.autonomous.demo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;

@Autonomous(name = "Demo: Red Side Far Position", group = "Demo Cluj-Napoca")
public class RedSideFarPosition extends AutonomousControl {

    @Override
    protected void run() {
        driveStraight(25);
        rotate(-90);
        int level = identifyElement();
        float distance = 50;
        if (level == 3 || level == 1) distance -= 20;
        useElevator(level);
        driveStraight(distance);
        rotate(90);
        useArm(false);
        distance = 22;
        if (level == 3) distance += 2;
        if (level == 2) distance++;
        driveStraight(distance);
        useClaws(false);
        driveStraight(-distance);
        rotate(-90);
        driveStraight(150);
        useArm(true);
        useElevator(4);
    }
}
