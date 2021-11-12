package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;


@TeleOp(name = "Basic: Movement", group = "Testing Purposes")
public class BasicMovement extends OpMode {

    RobotHardware robotHardware = new RobotHardware();

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robotHardware.init(hardwareMap);
        telemetry.addData("Status", "Initialized");


        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
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
        rightFront.setPower(0f);
        rightBack.setPower(0f);
        leftFront.setPower(0f);
        leftBack.setPower(0f);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        float rightTrigger = gamepad1.right_trigger;

        rightTrigger = Math.min(rightTrigger, 0.8f);


        // Send calculated power to wheels
        rightFront.setPower(rightTrigger);
        rightBack.setPower(rightTrigger);
        leftFront.setPower(rightTrigger);
        leftBack.setPower(rightTrigger);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "forward (%.2f)", rightTrigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        rightFront.setPower(0f);
        rightBack.setPower(0f);
        leftFront.setPower(0f);
        leftBack.setPower(0f);
    }

}
