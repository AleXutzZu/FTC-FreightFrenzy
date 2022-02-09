package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;

@Autonomous(name = "Plus formation", group = "Testing Purposes")
public class PlusFormation extends AutonomousControl {
    @Override
    protected void run() {
        driveStraight(100);
        driveStraight(-100);
        driveSideways(100);
        driveSideways(-100);
    }
}
