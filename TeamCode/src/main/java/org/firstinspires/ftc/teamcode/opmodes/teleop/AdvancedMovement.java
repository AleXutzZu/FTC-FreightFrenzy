package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Movements;


@TeleOp(name = "Advanced Movement", group = "Testing Purposes")
public class AdvancedMovement extends OpMode {
    private final float POWER_RATIO = 2f;
    private final float DEFAULT_POWER = 0.3f;
    private Movements robotMovements;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robotMovements = new Movements(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        robotMovements.stopMotors();
    }

    @Override
    public void loop() {
        /*
        Button mappings
        A = Handbrake
        RB = Rotate to the right
        LB = Rotate to the left
        D-pad Right = Slide right
        D-pad Left = Slide left
         */
        if (gamepad1.a) {
            robotMovements.stopMotors();
            return;
        }

        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                robotMovements.stopMotors();
                return;
            }
            if (gamepad1.right_bumper) robotMovements.rotateRight(DEFAULT_POWER);
            else robotMovements.rotateLeft(DEFAULT_POWER);
            return;
        }

        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            if (gamepad1.dpad_right && gamepad1.dpad_left) {
                robotMovements.stopMotors();
                return;
            }
            if (gamepad1.dpad_right) robotMovements.driveRight(DEFAULT_POWER);
            else robotMovements.driveLeft(DEFAULT_POWER);
            return;
        }
        /*
        Joysticks and triggers mappings
        LT - decelerate
        RT - accelerate
        Right Joystick - First Quadrant: Drive Diagonally Left Forwards
                       - Second Quadrant: Drive Diagonally Right Forwards
                       - Third Quadrant: Drive Diagonally Left Backwards
                       - Fourth Quadrant: Drive Diagonally Right Backwards
        Left Joystick - UP: Drive Straight Forward
                      - LEFT: Upper -> Turn left forwards
                              Lower -> Turn left backwards
                      - DOWN: Drive Straight Backwards
                      - RIGHT: Upper -> Turn right forwards
                               Lower -> Turn right backwards
         */
        float acceleration = (gamepad1.right_trigger - gamepad1.left_trigger) / POWER_RATIO;

    }

    @Override
    public void stop() {
        robotMovements.stopMotors();
    }
}
