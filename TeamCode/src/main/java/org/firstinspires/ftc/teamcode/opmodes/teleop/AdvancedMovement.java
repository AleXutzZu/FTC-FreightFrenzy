package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Movements;


@TeleOp(name = "Advanced Movement", group = "Testing Purposes")
public class AdvancedMovement extends OpMode {

    private final float POWER_RATIO = 2f;
    private final float DEFAULT_POWER = 0.5f;
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

        float forwardPower = gamepad1.right_trigger / POWER_RATIO;
        float backwardPower = -gamepad1.left_trigger / POWER_RATIO;
        float totalPower = forwardPower + backwardPower;

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_right || gamepad1.dpad_left) {
            boolean areMultipleButtonsPressed = (gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_right && !gamepad1.dpad_left)
                    || (gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right)
                    || (gamepad1.dpad_right && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left)
                    || (gamepad1.dpad_left && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_right);
            if (!areMultipleButtonsPressed) {
                robotMovements.stopMotors();
                return;
            }
            float additionalPower = Math.max(totalPower, 0f);
            if (gamepad1.left_bumper) robotMovements.rotateLeft(DEFAULT_POWER + additionalPower);
            if (gamepad1.right_bumper) robotMovements.rotateRight(DEFAULT_POWER + additionalPower);
            if (gamepad1.dpad_left) robotMovements.driveLeft(DEFAULT_POWER + additionalPower);
            if (gamepad1.dpad_right) robotMovements.driveRight(DEFAULT_POWER + additionalPower);
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

        boolean isLeftJoyStickActive = gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0;
        boolean isRightJoyStickActive = gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0;

        if ((isLeftJoyStickActive && isRightJoyStickActive) || (!isLeftJoyStickActive && !isRightJoyStickActive)) {
            robotMovements.stopMotors();
            return;
        }
        float additionalPower = Math.max(totalPower, 0f);
        /*
        Left joystick mappings
         */
        if (isLeftJoyStickActive) {
            /*
            Reverting the xOy axis
             */
            float horizontalCoordinate = -gamepad1.left_stick_x;
            float verticalCoordinate = -gamepad1.left_stick_y;


        }

        /*
        Right joystick mappings
         */
        if (isRightJoyStickActive) {
            /*
            Reverting the xOy axis
             */
            float horizontalCoordinate = -gamepad1.right_stick_x;
            float verticalCoordinate = -gamepad1.right_stick_y;
            /*
            Finding in which quadrant the controller is
             */
            if (horizontalCoordinate > 0f) {
                if (verticalCoordinate > 0f) {
                    robotMovements.driveDiagonallyRightForward(DEFAULT_POWER + additionalPower);
                } else robotMovements.driveDiagonallyRightBackward(DEFAULT_POWER + additionalPower);
            } else {
                if (verticalCoordinate > 0f) {
                    robotMovements.driveDiagonallyLeftForward(DEFAULT_POWER + additionalPower);
                } else robotMovements.driveDiagonallyLeftBackward(DEFAULT_POWER + additionalPower);
            }
        }
    }

    @Override
    public void stop() {
        robotMovements.stopMotors();
    }
}
