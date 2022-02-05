package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AutonomousControl;

@Autonomous(name = "Demo: Right Side Far Position", group = "Demo Cluj-Napoca")
public class RightSideFarPosition extends AutonomousControl {

    @Override
    protected void run() {
        driveStraight(1, 32);
        rotate(-1, 90);
        driveStraight(1, 230);
    }
}
