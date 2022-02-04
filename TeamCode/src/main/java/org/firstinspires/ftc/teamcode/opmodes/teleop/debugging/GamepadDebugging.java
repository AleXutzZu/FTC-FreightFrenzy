package org.firstinspires.ftc.teamcode.opmodes.teleop.debugging;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Gamepad Debugging", group = "Testing Purposes")
@Disabled
public class GamepadDebugging extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Stauts", "Initialized");
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
        telemetry.addData("Buttons", "Y: %s B: %s A: %s X: %s",
                Boolean.toString(gamepad1.y),
                Boolean.toString(gamepad1.b),
                Boolean.toString(gamepad1.a),
                Boolean.toString(gamepad1.x)
                ).addData("Other Buttons", "LJ: %s RJ: %s RB: %s LB: %s",
                        Boolean.toString(gamepad1.left_stick_button),
                        Boolean.toString(gamepad1.right_stick_button),
                        Boolean.toString(gamepad1.right_bumper),
                        Boolean.toString(gamepad1.left_bumper)
                ).addData("Triggers", "LT: %.2f, RT: %.2f", gamepad1.left_trigger, gamepad1.right_trigger)
                .addData("JoySticks", "Left -> X: %.2f Y: %.2f Right -> X: %.2f Y: %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y)
                .addData("D-PAD", "Up: %s Right: %s Down: %s Left: %s",
                        Boolean.toString(gamepad1.dpad_up),
                        Boolean.toString(gamepad1.dpad_right),
                        Boolean.toString(gamepad1.dpad_down),
                        Boolean.toString(gamepad1.dpad_left));

    }

    @Override
    public void stop() {
    }
}
