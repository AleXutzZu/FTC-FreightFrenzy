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
        float leftTrigger = gamepad1.left_trigger;

        leftTrigger = Math.min(leftTrigger, 0.8f);
        leftTrigger = -leftTrigger;
        rightTrigger = Math.min(rightTrigger, 0.8f);

        float forwardPower = leftTrigger + rightTrigger;
        // Send calculated power to wheels
        robotHardware.rightFrontMotor.setPower(forwardPower);
        robotHardware.rightBackMotor.setPower(forwardPower);
        robotHardware.leftFrontMotor.setPower(forwardPower);
        robotHardware.leftBackMotor.setPower(forwardPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "forward (%.2f)", forwardPower);
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
