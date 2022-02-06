package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class TeleOpControl extends OpMode {
    /**
     * Servo position to bring the arms up (Arm base Servo)
     */
    protected static final float ARM_UP = 1f;
    /**
     * Servo position to put the arm down (Arm base Servo)
     */
    protected static final float ARM_DOWN = 0.5f;
    /**
     * Servo position to close the claws (Claw servos)
     */
    protected static final float CLAWS_CLOSED = 1f;
    /**
     * Servo position to open the claws (Claw servos)
     */
    protected static final float CLAWS_OPENED = 0f;

    /**
     * Robot Hardware necessary for movement
     */
    protected final RobotHardware robotHardware = RobotHardware.getInstance();

    /**
     * Shows runtime since pressing start
     */
    protected final ElapsedTime runtime = new ElapsedTime();

    /**
     * Cooldown for opening/closing claws
     */
    protected final ElapsedTime clawButtonCooldown = new ElapsedTime();

    /**
     * Cooldown for using debug options
     */
    protected final ElapsedTime debugButtonCooldown = new ElapsedTime();

    /**
     * Whether or not the claws are open or not (true = open, false = closed)
     */
    protected boolean clawState = true;

    /**
     * Shows debug state
     * 0 = no debug
     * 1 = start debugging
     * 2 = freeze debug
     * 3 = clear debug then goes to state 0
     */
    protected int debugState = 0;

    /**
     * All possible directions a robot may take
     */
    private enum Direction {
        FORWARD, BACKWARD,
        STRAFE_LEFT, STRAFE_RIGHT,
        ROTATE_LEFT, ROTATE_RIGHT,
        DIAGONALLY_RIGHT_FORWARD, DIAGONALLY_RIGHT_BACKWARD,
        DIAGONALLY_LEFT_FORWARD, DIAGONALLY_LEFT_BACKWARD
    }

    /**
     * Drives the robot in a straight line by the given power
     *
     * @param motorPower positive value will result in forward movement, while negative in backward movement
     */
    protected void driveStraight(float motorPower) {
        drive(motorPower, (motorPower < 0f ? Direction.BACKWARD : Direction.FORWARD));
    }

    /**
     * Drives the robot sideways to the left or right by the given speed
     *
     * @param motorPower positive value will result in strafing to the left, while negative in strafing to the right
     */
    protected void driveSideways(float motorPower) {
        drive(motorPower, (motorPower > 0f ? Direction.STRAFE_LEFT : Direction.STRAFE_RIGHT));
    }

    /**
     * Drives the robot diagonally either forwards or backwards on the given diagonal with the specified power
     *
     * @param motorPower   positive power will result in forwards movement on the diagonal, while negative power will result
     *                     in backwards movement
     * @param leftDiagonal whether or not to use the left diagonal (false = right diagonal, true = left diagonal)
     */
    protected void driveDiagonally(float motorPower, boolean leftDiagonal) {
        if (leftDiagonal) {
            drive(motorPower, motorPower < 0f ? Direction.DIAGONALLY_LEFT_BACKWARD : Direction.DIAGONALLY_LEFT_FORWARD);
        } else {
            drive(motorPower, motorPower < 0f ? Direction.DIAGONALLY_RIGHT_BACKWARD : Direction.DIAGONALLY_RIGHT_FORWARD);
        }
    }

    /**
     * Rotates the robot with the given power
     *
     * @param motorPower Positive power will result in rotation to the left, while negative power will result in rotation to the right
     */
    protected void rotate(float motorPower) {
        drive(motorPower, (motorPower >= 0f ? Direction.ROTATE_LEFT : Direction.ROTATE_RIGHT));
    }

    /**
     * Helper function for TeleOp driving
     *
     * @param motorPower power to give to the motors
     * @param direction  non-null direction that tells which direction the robot should take
     */
    private void drive(float motorPower, @NonNull Direction direction) {
        float leftFrontPower = 0f, rightFrontPower = 0f, leftBackPower = 0f, rightBackPower = 0f;
        motorPower = Math.abs(motorPower);
        switch (direction) {
            case FORWARD:
                rightFrontPower = motorPower;
                leftFrontPower = motorPower;
                rightBackPower = motorPower;
                leftBackPower = motorPower;
                break;
            case BACKWARD:
                rightFrontPower = -motorPower;
                leftFrontPower = -motorPower;
                rightBackPower = -motorPower;
                leftBackPower = -motorPower;
                break;
            case STRAFE_LEFT:
                rightFrontPower = motorPower;
                leftFrontPower = -motorPower;
                rightBackPower = -motorPower;
                leftBackPower = motorPower;
                break;
            case STRAFE_RIGHT:
                rightFrontPower = -motorPower;
                leftFrontPower = motorPower;
                rightBackPower = motorPower;
                leftBackPower = -motorPower;
                break;
            case ROTATE_LEFT:
                rightFrontPower = -motorPower;
                leftFrontPower = motorPower;
                rightBackPower = -motorPower;
                leftBackPower = motorPower;
                break;
            case ROTATE_RIGHT:
                rightFrontPower = motorPower;
                leftFrontPower = -motorPower;
                rightBackPower = motorPower;
                leftBackPower = -motorPower;
                break;
            case DIAGONALLY_RIGHT_FORWARD:
                rightFrontPower = motorPower;
                leftFrontPower = 0;
                rightBackPower = 0;
                leftBackPower = motorPower;
                break;
            case DIAGONALLY_RIGHT_BACKWARD:
                rightFrontPower = 0;
                leftFrontPower = -motorPower;
                rightBackPower = -motorPower;
                leftBackPower = 0;
                break;
            case DIAGONALLY_LEFT_FORWARD:
                rightFrontPower = 0;
                leftFrontPower = motorPower;
                rightBackPower = motorPower;
                leftBackPower = 0;
                break;
            case DIAGONALLY_LEFT_BACKWARD:
                rightFrontPower = -motorPower;
                leftFrontPower = 0;
                rightBackPower = 0;
                leftBackPower = -motorPower;
                break;
        }
        robotHardware.getRightFrontMotor().setPower(rightFrontPower);
        robotHardware.getRightBackMotor().setPower(rightBackPower);
        robotHardware.getLeftFrontMotor().setPower(leftFrontPower);
        robotHardware.getLeftBackMotor().setPower(leftBackPower);
    }

    /**
     * Brings the motors to a halt
     */
    protected void stopMotors() {
        robotHardware.getRightFrontMotor().setPower(0f);
        robotHardware.getRightBackMotor().setPower(0f);
        robotHardware.getLeftFrontMotor().setPower(0f);
        robotHardware.getLeftBackMotor().setPower(0f);
    }

    /**
     * Operates the arm on the robot.
     * It brings the arm up in the ARM_UP position and brings it down to the ARM_DOWN position
     *
     * @param armUp whether or not to bring the arm up or down (true = bring up, false = put down)
     * @see TeleOpControl#ARM_UP
     * @see TeleOpControl#ARM_DOWN
     */
    protected void useArm(boolean armUp) {
        robotHardware.getArmBase().setPosition(armUp ? ARM_UP : ARM_DOWN);
    }

    /**
     * Operates the claws on the robot.
     * It closes the claws by putting them in the CLAWS_CLOSED position and opens them by putting the servos
     * in the CLAWS_OPENED position
     *
     * @param openClaws whether or not the claws should open or not (true = open claws, false = close claws)
     * @see TeleOpControl#CLAWS_OPENED
     * @see TeleOpControl#CLAWS_CLOSED
     */
    protected void useClaws(boolean openClaws) {
        if (openClaws) {
            robotHardware.getRightClaw().setPosition(CLAWS_OPENED);
            robotHardware.getLeftClaw().setPosition(CLAWS_OPENED);
        } else {
            robotHardware.getLeftClaw().setPosition(CLAWS_CLOSED);
            robotHardware.getRightClaw().setPosition(CLAWS_CLOSED);
        }
    }

    /**
     * Operates the elevator on the front of the robot with the specified speed
     *
     * @param motorPower Positive value will bring the elevator up, while a negative
     *                   value will bring it down
     */
    protected void useElevator(float motorPower) {
        robotHardware.getElevatorMotor().setPower(motorPower);
    }

    /**
     * Operates the wheel on the back side of the robot with the specified speed
     *
     * @param motorPower Positive value will rotate the robot in left direction,
     *                   while negative value in the right direction
     */
    protected void useWheelMotor(float motorPower) {
        robotHardware.getWheelMotor().setPower(motorPower);
    }

    @Override
    public void start() {
        runtime.reset();
        clawButtonCooldown.reset();
        debugButtonCooldown.reset();
        telemetry.addData("Runtime", runtime.toString());
    }

    @Override
    public void init() {
        robotHardware.init(hardwareMap);
        useClaws(false);
        useArm(true);
    }
}
