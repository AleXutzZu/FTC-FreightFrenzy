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
        Movements robotMovements = Movements.getInstance();
        robotMovements.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()){
            //square path
            robotMovements.driveForward(1f, 150f);
            robotMovements.rotateRight(1f, 90f);
            robotMovements.driveForward(1f, 150f);
            robotMovements.rotateRight(1f, 90f);
            robotMovements.driveForward(1f, 150f);
            robotMovements.rotateRight(1f, 90f);
            robotMovements.driveForward(1f, 150f);
        }
    }
}
