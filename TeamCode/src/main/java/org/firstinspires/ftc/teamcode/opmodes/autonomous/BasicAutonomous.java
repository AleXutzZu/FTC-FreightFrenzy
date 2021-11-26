package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Movements;


@Autonomous(name = "Basic Autonomous", group= "Testing Purposes")
public class BasicAutonomous extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Movements robotMovements = new Movements(hardwareMap);

        waitForStart();

        robotMovements.driveForward(1f, 100);
        robotMovements.driveBackward(1f, 100);
        robotMovements.driveRight(1f, 100);
        robotMovements.driveLeft(1f, 100);
        robotMovements.stopMotors();
    }
}
