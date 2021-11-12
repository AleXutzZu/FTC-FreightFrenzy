package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;


@TeleOp(name = "Basic: Movement", group = "Testing Purposes")
public class BasicMovement extends OpMode {

    RobotHardware robotHardware = new RobotHardware();

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robotHardware.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(0f);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        float rightTrigger = gamepad1.right_trigger;

        rightTrigger = Math.min(rightTrigger, 0.8f);


        // Send calculated power to wheels
        robotHardware.rightFrontMotor.setPower(rightTrigger);
        robotHardware.rightBackMotor.setPower(rightTrigger);
        robotHardware.leftFrontMotor.setPower(rightTrigger);
        robotHardware.leftBackMotor.setPower(rightTrigger);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "forward (%.2f)", rightTrigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(0f);
    }

}
