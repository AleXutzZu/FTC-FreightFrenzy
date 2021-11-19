package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Basic Autonomous", group= "Testing Purposes")
public class BasicAutonomous extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {

    }
}
