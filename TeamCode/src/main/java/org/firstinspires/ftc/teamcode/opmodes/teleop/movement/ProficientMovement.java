package org.firstinspires.ftc.teamcode.opmodes.teleop.movement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.TeleOpControl;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "Proficient Movement", group = "Regio")
@Disabled
public final class ProficientMovement extends TeleOpControl {
    /**
     * Cooldown for opening/closing claws
     */
    private final ElapsedTime clawButtonCooldown = new ElapsedTime();

    @Override
    protected void drive() {

        /*
        Button mappings
        A = Handbrake
        RB = Rotate to the right
        LB = Rotate to the left
        D-pad Right = Slide right
        D-pad Left = Slide left
         */

        if (gamepad1.a) {
            stopMotors();
            return;
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_right || gamepad1.dpad_left) {
            boolean areMultipleButtonsPressed = (gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_right && !gamepad1.dpad_left)
                    || (gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right)
                    || (gamepad1.dpad_right && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left)
                    || (gamepad1.dpad_left && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_right);
            if (!areMultipleButtonsPressed) {
                stopMotors();
                return;
            }
            if (gamepad1.left_bumper) rotate(Constants.ROTATION_POWER);
            if (gamepad1.right_bumper) rotate(-Constants.ROTATION_POWER);
            if (gamepad1.dpad_left) driveSideways(Constants.SLIDING_POWER);
            if (gamepad1.dpad_right) driveSideways(-Constants.SLIDING_POWER);
            return;
        }

        double forwardPower = gamepad1.right_trigger / Constants.POWER_RATIO;
        double backwardPower = -gamepad1.left_trigger / Constants.POWER_RATIO;
        double totalPower = forwardPower + backwardPower;

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

        if (isLeftJoyStickActive && isRightJoyStickActive) {
            stopMotors();
            return;
        }
        /*
        Right joystick mappings
         */
        if (isRightJoyStickActive) {
            /*
            Reverting the xOy axis
             */
            double horizontalCoordinate = -gamepad1.right_stick_x;
            double verticalCoordinate = -gamepad1.right_stick_y;
            /*
            Finding in which quadrant the controller is
             */
            if (horizontalCoordinate > 0) {
                if (verticalCoordinate > 0) {
                    driveDiagonally(Constants.DIAGONAL_DRIVING_POWER, false);
                } else {
                    driveDiagonally(-Constants.DIAGONAL_DRIVING_POWER, false);
                }
            } else {
                if (verticalCoordinate > 0) {
                    driveDiagonally(Constants.DIAGONAL_DRIVING_POWER, true);
                } else {
                    driveDiagonally(-Constants.DIAGONAL_DRIVING_POWER, true);
                }
            }
            return;
        }
        /*
        Left joystick mappings
         */
        if (!isLeftJoyStickActive) {
            if (totalPower == 0) {
                stopMotors();
                return;
            }
            driveStraight(totalPower);
        } else {
            /*
            Reverting the xOy axis
             */
            double verticalCoordinate = -gamepad1.left_stick_y;

            double boostPower = verticalCoordinate / Constants.POWER_RATIO;
            //UP
            if (boostPower >= 0f) {
                if (totalPower > 0f) {
                    driveStraight(boostPower + totalPower);
                } else {
                    stopMotors();
                }
            }
            //DOWN
            else {
                if (totalPower < 0f) {
                    driveStraight(boostPower + totalPower);
                } else {
                    stopMotors();
                }
            }
        }
    }

    @Override
    protected void useLimbs() {
        /*
        Key mappings
        Right Trigger - Rotate wheel to the right
        Left Trigger - Rotate wheel to the left
        Right joystick - bring up/down the elevator
        dpad up/down - put up/down the arm
        LB - open/close claws
         */
        //Wheel rotation
        float wheelPower = -gamepad2.right_trigger + gamepad2.left_trigger;
        if (wheelPower != 0) {
            useWheelMotor(wheelPower);
            return;
        } else useWheelMotor(0);

        //Elevator controls
        float elevatorY = -gamepad2.right_stick_y;
        boolean isRightJoyStickActive = (elevatorY != 0);
        if (isRightJoyStickActive) {
            useElevator(elevatorY);
            return;
        } else useElevator(0);

        //Arm controls
        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                useArm(true);
                return;
            }
            if (!gamepad2.dpad_up) {
                useArm(false);
                return;
            }
        }

        //Claw control
        if (gamepad2.b) {
            if (clawButtonCooldown.time() >= 0.5) {
                clawButtonCooldown.reset();
                useClaws();
            }
        }
    }
}
