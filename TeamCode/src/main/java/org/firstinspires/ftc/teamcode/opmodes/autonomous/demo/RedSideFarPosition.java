package org.firstinspires.ftc.teamcode.opmodes.autonomous.demo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;

@Autonomous(name = "Demo: Red Side Far Position", group = "Demo Cluj-Napoca")
public class RedSideFarPosition extends AutonomousControl {

    @Override
    protected void run() {
        driveStraight(32);
        rotate(-90);
        driveStraight(230);
    }
}
