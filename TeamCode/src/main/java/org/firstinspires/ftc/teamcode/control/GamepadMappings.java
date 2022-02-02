package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Gamepads;
import org.firstinspires.ftc.teamcode.util.LimbPosition;
import org.firstinspires.ftc.teamcode.util.RobotMovementControls;

public class GamepadMappings extends Gamepads {
    private boolean debug = false;
    private final Gamepad gamepad1, gamepad2;
    private final ElapsedTime rotateWheelKeyCooldown = new ElapsedTime();
    private final ElapsedTime debugKeyCooldown = new ElapsedTime();

    public GamepadMappings(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        rotateWheelKeyCooldown.reset();
        debugKeyCooldown.reset();
    }

    @Override
    public Direction drive() {
        /*
        Debug Key = Y
         */
        if (gamepad1.y) {
            if (debugKeyCooldown.time() >= 2f) {
                debugKeyCooldown.reset();
                debug = !debug;
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
            robotMovements.stopMotors();
            return Direction.IDLE;
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.dpad_right || gamepad1.dpad_left) {
            boolean areMultipleButtonsPressed = (gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_right && !gamepad1.dpad_left)
                    || (gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right)
                    || (gamepad1.dpad_right && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left)
                    || (gamepad1.dpad_left && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_right);
            if (!areMultipleButtonsPressed) {
                robotMovements.stopMotors();
                return Direction.IDLE;
            }
            if (gamepad1.left_bumper) {
                robotMovements.rotateLeft(RobotMovementControls.ROTATION_POWER);
                direction = Direction.ROTATE_LEFT;
            }
            if (gamepad1.right_bumper) {
                robotMovements.rotateRight(RobotMovementControls.ROTATION_POWER);
                direction = Direction.ROTATE_RIGHT;
            }
            if (gamepad1.dpad_left) {
                robotMovements.driveLeft(RobotMovementControls.SLIDING_POWER);
                direction = Direction.SLIDE_LEFT;
            }
            if (gamepad1.dpad_right) {
                robotMovements.driveRight(RobotMovementControls.SLIDING_POWER);
                direction = Direction.SLIDE_RIGHT;
            }
            return direction;
        }

        float forwardPower = gamepad1.right_trigger / RobotMovementControls.POWER_RATIO;
        float backwardPower = -gamepad1.left_trigger / RobotMovementControls.POWER_RATIO;
        float totalPower = forwardPower + backwardPower;

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
            robotMovements.stopMotors();
            return Direction.IDLE;
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
                    robotMovements.driveDiagonallyRightForward(RobotMovementControls.DIAGONAL_DRIVING_POWER);
                    return Direction.DIAGONALLY_RIGHT_FORWARD;
                } else {
                    robotMovements.driveDiagonallyRightBackward(RobotMovementControls.DIAGONAL_DRIVING_POWER);
                    return Direction.DIAGONALLY_RIGHT_BACKWARD;
                }
            } else {
                if (verticalCoordinate > 0f) {
                    robotMovements.driveDiagonallyLeftForward(RobotMovementControls.DIAGONAL_DRIVING_POWER);
                    return Direction.DIAGONALLY_LEFT_FORWARD;
                } else {
                    robotMovements.driveDiagonallyLeftBackward(RobotMovementControls.DIAGONAL_DRIVING_POWER);
                    return Direction.DIAGONALLY_LEFT_BACKWARD;
                }
            }
        }
        /*
        Left joystick mappings
         */
        if (!isLeftJoyStickActive) {
            if (totalPower > 0f) {
                robotMovements.driveForward(totalPower);
                return Direction.STRAIGHT_FORWARD;
            }
            if (totalPower == 0f) {
                robotMovements.stopMotors();
                return Direction.IDLE;
            }
            robotMovements.driveBackward(-totalPower);
            return Direction.STRAIGHT_BACKWARD;
        } else {
            /*
            Reverting the xOy axis
             */
            float horizontalCoordinate = -gamepad1.left_stick_x;
            float verticalCoordinate = -gamepad1.left_stick_y;

            /*
            Graphing the axis
                                         g(x) = -x     / \ (0, 1)      f(x) = x
                                          \             |             /
                                           \            |            /
                                            \           |           /
                                             \          |          /
                                              \         |         /
                                               \        |        /
                                                \       |       /
                                                 \      |      /
                                                  \     |     /
                                                   \    |    /
                                                    \   |   /
                                                     \  |  /
                                                      \ | /
            (-1, 0) <-----------------------------------+-----------------------------------> (1, 0)
                                                      / | \
                                                     /  |  \
                                                    /   |   \
                                                   /    |    \
                                                  /     |     \
                                                 /      |      \
                                                /       |       \
                                               /        |        \
                                              /         |         \
                                             /          |          \
                                            /           |           \
                                           /            |            \
                                          /             |             \
                                                       \ / (0, -1)
            We deduce the following laws:
            For any given pair (x,y) on the xOy axis the following will be true:
            If f(x) < y then (x, y) is above the graph of f(x) otherwise it is under
            If g(x) < y then (x, y) is above the graph of g(x) otherwise it is under
            Given these conditions, we can deduce the conditions for each zone:
                UP : Above g and f
                RIGHT: Above g but below f
                DOWN: Below g and f
                LEFT: Below g but above f
            These can be transformed into 4 conditions:
                UP : x <= y && -x <= y
                RIGHT: x > y && -x < y
                DOWN: x >= y && -x >= y
                LEFT: x < y && -x > y
             */
            float basePower;
            //UP
            if (horizontalCoordinate <= verticalCoordinate && -horizontalCoordinate <= verticalCoordinate) {
                basePower = verticalCoordinate / RobotMovementControls.POWER_RATIO;
                if (totalPower > 0f) {
                    robotMovements.driveForward(basePower + totalPower);
                    direction = Direction.STRAIGHT_FORWARD;
                } else {
                    robotMovements.stopMotors();
                    direction = Direction.IDLE;
                }
                return direction;
            }
            //DOWN
            if (horizontalCoordinate >= verticalCoordinate && -horizontalCoordinate >= verticalCoordinate) {
                basePower = -verticalCoordinate / RobotMovementControls.POWER_RATIO;
                if (totalPower < 0f) {
                    robotMovements.driveBackward(basePower - totalPower);
                    direction = Direction.STRAIGHT_BACKWARD;
                } else {
                    robotMovements.stopMotors();
                    direction = Direction.IDLE;
                }
                return direction;
            }
            //RIGHT
            if (horizontalCoordinate > verticalCoordinate && -horizontalCoordinate < verticalCoordinate) {
                basePower = horizontalCoordinate / RobotMovementControls.POWER_RATIO;
                if (totalPower >= 0f) {
                    robotMovements.steerForward(basePower / RobotMovementControls.POWER_RATIO, basePower + totalPower);
                    direction = Direction.TURN_RIGHT_FORWARDS;
                } else {
                    robotMovements.steerBackward(basePower - totalPower, basePower / RobotMovementControls.POWER_RATIO);
                    direction = Direction.TURN_RIGHT_BACKWARDS;
                }
                return direction;
            }
            //LEFT
            if (horizontalCoordinate < verticalCoordinate && -horizontalCoordinate > verticalCoordinate) {
                basePower = -horizontalCoordinate / RobotMovementControls.POWER_RATIO;
                if (totalPower >= 0f) {
                    robotMovements.steerForward(basePower + totalPower, basePower / RobotMovementControls.POWER_RATIO);
                    direction = Direction.TURN_LEFT_FORWARDS;
                } else {
                    robotMovements.steerBackward(basePower / RobotMovementControls.POWER_RATIO, basePower - totalPower);
                    direction = Direction.TURN_LEFT_BACKWARDS;
                }
                return direction;
            }
            return Direction.IDLE;
        }
    }

    @Override
    public LimbPosition useLimbs() {
        /*
        Key mappings
        A - Turn on/off wheel (hold to rotate)
        Right joystick - bring up/down the elevator
        dpad up/down - put up/down the arm
        LB - open/close claws
         */
        //Wheel rotation
        if (gamepad2.a) {
            robotLimbs.rotateWheel();
            return LimbPosition.WHEEL_ROTATING;
        } else robotLimbs.pauseWheelMotor();

        //Elevator controls
        float elevatorY = -gamepad2.right_stick_y;
        boolean isRightJoyStickActive = (elevatorY != 0f);
        if (isRightJoyStickActive) {
            robotLimbs.useElevator(elevatorY / RobotMovementControls.POWER_RATIO);
            if (elevatorY > 0f) {
                return LimbPosition.ELEVATOR_UP;
            } else {
                return LimbPosition.ELEVATOR_DOWN;
            }
        } else robotLimbs.useElevator(0f);

        //Arm controls
        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                robotLimbs.useArm(true);
                return LimbPosition.ARM_UP;
            }
            if (!gamepad2.dpad_up) {
                robotLimbs.useArm(false);
                return LimbPosition.ARM_DOWN;
            }
        }

        //Claw control
        if (gamepad2.b) {
            if (rotateWheelKeyCooldown.time() >= 1f) {
                rotateWheelKeyCooldown.reset();
                if (robotLimbs.isClaws()) {
                    robotLimbs.setClaws(false);
                    robotLimbs.useClaws(false);
                    return LimbPosition.CLAWS_CLOSED;
                } else {
                    robotLimbs.setClaws(true);
                    robotLimbs.useClaws(true);
                    return LimbPosition.CLAWS_OPEN;
                }
            }
        }
        return LimbPosition.IDLE;
    }

    public boolean isDebug() {
        return debug;
    }

    public ElapsedTime getRotateWheelKeyCooldown() {
        return rotateWheelKeyCooldown;
    }

    public ElapsedTime getDebugKeyCooldown() {
        return debugKeyCooldown;
    }

    /**
     * Debug function
     *
     * @return the power in the left front motor (double)
     */
    public double getLeftFrontMotorPower() {
        return robotMovements.getRobotHardware().getLeftFrontMotor().getPower();
    }

    /**
     * Debug function
     *
     * @return the power in the right front motor (double)
     */
    public double getRightFrontMotorPower() {
        return robotMovements.getRobotHardware().getRightFrontMotor().getPower();
    }

    /**
     * Debug function
     *
     * @return the power in the left back motor (double)
     */
    public double getLeftBackMotorPower() {
        return robotMovements.getRobotHardware().getLeftBackMotor().getPower();
    }

    /**
     * Debug function
     *
     * @return the power in the right back motor (double)
     */
    public double getRightBackMotorPower() {
        return robotMovements.getRobotHardware().getRightBackMotor().getPower();
    }

    /**
     * Debug and helper function
     *
     * @return the position in ticks at which the motor is at (int)
     */
    public int getElevatorMotorTicks() {
        return robotLimbs.getRobotHardware().getElevatorMotor().getCurrentPosition();
    }
}
