package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AutonomousControl;

@Autonomous(name = "Demo: Left Side Close Position", group = "Demo Cluj-Napoca")
public class LeftSideClosePosition extends AutonomousControl {

    @Override
    protected void run() {
        driveStraight(1, 32);
        rotate(90);
        driveStraight(1, 100);
    }
}
