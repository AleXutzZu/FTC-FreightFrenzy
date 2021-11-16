package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.util.Direction;


@TeleOp(name = "Basic: Movement", group = "Testing Purposes")
public class BasicMovement extends OpMode {

    private Control robotControl;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robotControl = Control.getInstance();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        robotControl.stopMotors();
    }

    @Override
    public void loop() {
        Direction direction;
        float rightTrigger = gamepad1.right_trigger;
        float leftTrigger = gamepad1.left_trigger;

        leftTrigger = -Math.min(leftTrigger, 0.8f);
        rightTrigger = Math.min(rightTrigger, 0.8f);

        float power = leftTrigger + rightTrigger;
        // Send calculated power to wheels
        if (power < 0) robotControl.driveBackward(-power);
        else robotControl.driveForward(power);
        direction = (power < 0 ? Direction.STRAIGHT_BACKWARD : Direction.STRAIGHT_FORWARD);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Direction", "%s -> power : %.2f", direction, power);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {
        robotControl.stopMotors();
    }
}
