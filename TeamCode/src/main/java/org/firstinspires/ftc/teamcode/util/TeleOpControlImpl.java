package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "New TeleOp System Attempt", group = "Testing Purposes")
@Disabled
public class TeleOpControlImpl extends TeleOpControl {

    @Override
    protected Direction drive() {
        /*
        Debug Key = Y
         */
        if (gamepad1.y) {
            if (debugButtonCooldown.time() >= 0.5) {
                debugButtonCooldown.reset();
                debugState++;
            }
        }
        Direction direction = null;
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
            return Direction.IDLE;
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_right || gamepad1.dpad_left) {
            boolean areMultipleButtonsPressed = (gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_right && !gamepad1.dpad_left)
                    || (gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right)
                    || (gamepad1.dpad_right && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left)
                    || (gamepad1.dpad_left && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_right);
            if (!areMultipleButtonsPressed) {
                stopMotors();
                return Direction.IDLE;
            }
            if (gamepad1.left_bumper) {
                rotate(ROTATION_POWER);
                direction = Direction.ROTATE_LEFT;
            }
            if (gamepad1.right_bumper) {
                rotate(-ROTATION_POWER);
                direction = Direction.ROTATE_RIGHT;
            }
            if (gamepad1.dpad_left) {
                driveSideways(SLIDING_POWER);
                direction = Direction.STRAFE_LEFT;
            }
            if (gamepad1.dpad_right) {
                driveSideways(-SLIDING_POWER);
                direction = Direction.STRAFE_RIGHT;
            }
            return direction;
        }

        double forwardPower = gamepad1.right_trigger / POWER_RATIO;
        double backwardPower = -gamepad1.left_trigger / POWER_RATIO;
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
            return Direction.IDLE;
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
                    driveDiagonally(DIAGONAL_DRIVING_POWER, false);
                    return Direction.DIAGONALLY_RIGHT_FORWARD;
                } else {
                    driveDiagonally(-DIAGONAL_DRIVING_POWER, false);
                    return Direction.DIAGONALLY_RIGHT_BACKWARD;
                }
            } else {
                if (verticalCoordinate > 0) {
                    driveDiagonally(RobotMovementControls.DIAGONAL_DRIVING_POWER, true);
                    return Direction.DIAGONALLY_LEFT_FORWARD;
                } else {
                    driveDiagonally(-RobotMovementControls.DIAGONAL_DRIVING_POWER, true);
                    return Direction.DIAGONALLY_LEFT_BACKWARD;
                }
            }
        }
        /*
        Left joystick mappings
         */
        if (!isLeftJoyStickActive) {
            if (totalPower == 0) {
                stopMotors();
                return Direction.IDLE;
            }

            driveStraight(totalPower);

            return (totalPower < 0 ? Direction.BACKWARD : Direction.FORWARD);
        } else {
            /*
            Reverting the xOy axis
             */
            double verticalCoordinate = -gamepad1.left_stick_y;

            double boostPower = verticalCoordinate / POWER_RATIO;
            //UP
            if (boostPower >= 0f) {
                if (totalPower > 0f) {
                    driveStraight(boostPower + totalPower);
                    direction = Direction.FORWARD;
                } else {
                    stopMotors();
                    direction = Direction.IDLE;
                }
            }
            //DOWN
            else {
                if (totalPower < 0f) {
                    driveStraight(boostPower + totalPower);
                    direction = Direction.BACKWARD;
                } else {
                    stopMotors();
                    direction = Direction.IDLE;
                }
            }
            return direction;
        }
    }

    @Override
    protected Limb useLimbs() {
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
            return Limb.ROTATING_WHEEL;
        } else useWheelMotor(0);

        //Elevator controls
        float elevatorY = -gamepad2.right_stick_y;
        boolean isRightJoyStickActive = (elevatorY != 0);
        if (isRightJoyStickActive) {
            useElevator(elevatorY);
            if (elevatorY > 0) {
                return Limb.ELEVATOR_UP;
            } else {
                return Limb.ELEVATOR_DOWN;
            }
        } else useElevator(0);

        //Arm controls
        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                useArm(true);
                return Limb.ARM_UP;
            }
            if (!gamepad2.dpad_up) {
                useArm(false);
                return Limb.ARM_DOWN;
            }
        }

        //Claw control
        if (gamepad2.b) {
            if (clawButtonCooldown.time() >= 0.5) {
                clawButtonCooldown.reset();
                if (clawState) {
                    clawState = false;
                    useClaws(false);
                    return Limb.CLAWS_CLOSED;
                } else {
                    clawState = true;
                    useClaws(true);
                    return Limb.CLAWS_OPEN;
                }
            }
        }
        return Limb.IDLE;
    }
}
