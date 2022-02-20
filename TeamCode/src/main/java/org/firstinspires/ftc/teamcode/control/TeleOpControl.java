package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Constants;

public abstract class TeleOpControl extends LinearOpMode {
    /**
     * Robot Hardware necessary for movement
     */
    protected final RobotHardware robotHardware = RobotHardware.getInstance();

    /**
     * Emergency button in case the OpMode crashes and the elevator remains stuck
     */
    private final ElapsedTime emergencyButtonCooldown = new ElapsedTime();

    /**
     * In case of an emergency, allow the operator to bypass the elevator limits to fix it (needed when an OpMode crashes)
     */
    private boolean bypassElevator = false;
    /**
     * Whether or not the claws are open or not (true = open, false = closed)
     */
    private boolean clawState = true;

    /*private Telemetry.Item drivingTelemetry;
    private Telemetry.Item leftFrontMotorTelemetry;
    private Telemetry.Item rightFrontMotorTelemetry;
    private Telemetry.Item leftBackMotorTelemetry;
    private Telemetry.Item rightBackMotorTelemetry;
    private Telemetry.Item runtimeTelemetry;
    private Telemetry.Item craneTelemetry;
    private Telemetry.Item armTelemetry;
    private Telemetry.Item clawsTelemetry;
    private Telemetry.Item wheelMotorTelemetry;
    private Telemetry.Item elevatorMotorTelemetry;*/

    /**
     * All possible directions a robot may take
     */
    private enum Direction {
        FORWARD, BACKWARD,
        STRAFE_LEFT, STRAFE_RIGHT,
        ROTATE_LEFT, ROTATE_RIGHT,
        DIAGONALLY_RIGHT_FORWARD, DIAGONALLY_RIGHT_BACKWARD,
        DIAGONALLY_LEFT_FORWARD, DIAGONALLY_LEFT_BACKWARD,
        IDLE
    }

    /**
     * All possible limb functions
     */
    private enum Limb {
        ELEVATOR_UP, ELEVATOR_DOWN,
        ROTATING_WHEEL,
        CLAWS_OPEN, CLAWS_CLOSED,
        ARM_UP, ARM_DOWN,
        IDLE
    }

    /**
     * Drives the robot in a straight line by the given power
     *
     * @param motorPower positive value will result in forward movement, while negative in backward movement
     */
    protected void driveStraight(double motorPower) {
        drive(motorPower, (motorPower < 0f ? Direction.BACKWARD : Direction.FORWARD));
    }

    /**
     * Drives the robot sideways to the left or right by the given speed
     *
     * @param motorPower positive value will result in strafing to the left, while negative in strafing to the right
     */
    protected void driveSideways(double motorPower) {
        drive(motorPower, (motorPower > 0f ? Direction.STRAFE_LEFT : Direction.STRAFE_RIGHT));
    }

    /**
     * Drives the robot diagonally either forwards or backwards on the given diagonal with the specified power
     *
     * @param motorPower   positive power will result in forwards movement on the diagonal, while negative power will result
     *                     in backwards movement
     * @param leftDiagonal whether or not to use the left diagonal (false = right diagonal, true = left diagonal)
     */
    protected void driveDiagonally(double motorPower, boolean leftDiagonal) {
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
    protected void rotate(double motorPower) {
        drive(motorPower, (motorPower >= 0f ? Direction.ROTATE_LEFT : Direction.ROTATE_RIGHT));
    }

    /**
     * Helper function for TeleOp driving
     *
     * @param motorPower power to give to the motors
     * @param direction  non-null direction that tells which direction the robot should take
     */
    private void drive(double motorPower, @NonNull Direction direction) {
        double leftFrontPower = 0, rightFrontPower = 0, leftBackPower = 0, rightBackPower = 0;
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
                rightFrontPower = -motorPower;
                leftFrontPower = motorPower;
                rightBackPower = motorPower;
                leftBackPower = -motorPower;
                break;
            case STRAFE_RIGHT:
                rightFrontPower = motorPower;
                leftFrontPower = -motorPower;
                rightBackPower = -motorPower;
                leftBackPower = motorPower;
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
                rightFrontPower = 0;
                leftFrontPower = -motorPower;
                rightBackPower = -motorPower;
                leftBackPower = 0;
                break;
            case DIAGONALLY_RIGHT_BACKWARD:
                rightFrontPower = motorPower;
                leftFrontPower = 0;
                rightBackPower = 0;
                leftBackPower = motorPower;
                break;
            case DIAGONALLY_LEFT_FORWARD:
                rightFrontPower = -motorPower;
                leftFrontPower = 0;
                rightBackPower = 0;
                leftBackPower = -motorPower;
                break;
            case DIAGONALLY_LEFT_BACKWARD:
                rightFrontPower = 0;
                leftFrontPower = motorPower;
                rightBackPower = motorPower;
                leftBackPower = 0;
                break;
            case IDLE:
                rightFrontPower = 0f;
                leftFrontPower = 0f;
                rightBackPower = 0f;
                leftBackPower = 0f;
                break;
        }
        robotHardware.getRightFrontMotor().setPower(rightFrontPower);
        robotHardware.getRightBackMotor().setPower(rightBackPower);
        robotHardware.getLeftFrontMotor().setPower(leftFrontPower);
        robotHardware.getLeftBackMotor().setPower(leftBackPower);

        double averagePower = (robotHardware.getLeftFrontMotor().getPower() + robotHardware.getLeftBackMotor().getPower() + robotHardware.getRightBackMotor().getPower() + robotHardware.getRightFrontMotor().getPower()) / 4d;

        /*this.drivingTelemetry.setValue("dir %s avg. pow %.2f", direction, averagePower);
        this.leftFrontMotorTelemetry.setValue("dir %s pow %.2f pos %d", robotHardware.getLeftFrontMotor().getDirection(), robotHardware.getLeftFrontMotor().getPower(), robotHardware.getLeftFrontMotor().getCurrentPosition());
        this.rightFrontMotorTelemetry.setValue("dir %s pow %.2f pos %d", robotHardware.getRightFrontMotor().getDirection(), robotHardware.getRightFrontMotor().getPower(), robotHardware.getRightFrontMotor().getCurrentPosition());
        this.leftBackMotorTelemetry.setValue("dir %s pow %.2f pos %d", robotHardware.getLeftBackMotor().getDirection(), robotHardware.getLeftBackMotor().getPower(), robotHardware.getLeftBackMotor().getCurrentPosition());
        this.rightBackMotorTelemetry.setValue("dir %s pow %.2f pos %d", robotHardware.getRightBackMotor().getDirection(), robotHardware.getRightBackMotor().getPower(), robotHardware.getRightBackMotor().getCurrentPosition());
        telemetry.update();*/
    }

    /**
     * Brings the motors to a halt
     */
    protected void stopMotors() {
        drive(0, Direction.IDLE);
    }

    /**
     * Operates the arm on the robot.
     * It brings the arm up in the ARM_UP position and brings it down to the ARM_DOWN position
     *
     * @param armUp whether or not to bring the arm up or down (true = bring up, false = put down)
     * @see Constants#ARM_UP
     * @see Constants#ARM_DOWN
     */
    protected void useArm(boolean armUp) {
        robotHardware.getArmBase().setPosition(armUp ? Constants.ARM_UP : Constants.ARM_DOWN);

        /*craneTelemetry.setValue(armUp ? Limb.ARM_UP : Limb.ARM_DOWN);
        armTelemetry.setValue(armUp ? "up" : "dwn");
        telemetry.update();*/
    }

    /**
     * Operates the claws on the robot.
     * It closes the claws by putting them in the CLAWS_CLOSED position and opens them by putting the servos
     * in the CLAWS_OPENED position
     *
     * @see Constants#CLAWS_OPENED
     * @see Constants#CLAWS_CLOSED
     */
    protected void useClaws() {
        if (clawState) {
            clawState = false;
            robotHardware.getLeftClaw().setPosition(Constants.CLAWS_CLOSED);
            robotHardware.getRightClaw().setPosition(Constants.CLAWS_CLOSED);
        } else {
            clawState = true;
            robotHardware.getRightClaw().setPosition(Constants.CLAWS_OPENED);
            robotHardware.getLeftClaw().setPosition(Constants.CLAWS_OPENED);
        }

        /*craneTelemetry.setValue(clawState ? Limb.CLAWS_CLOSED : Limb.CLAWS_OPEN);
        clawsTelemetry.setValue(clawState ? "opn" : "cls");
        telemetry.update();*/
    }

    /**
     * Operates the elevator on the front of the robot with the specified speed
     *
     * @param motorPower Positive value will bring the elevator up, while a negative
     *                   value will bring it down
     */
    protected void useElevator(double motorPower) {
        if (bypassElevator) {
            robotHardware.getElevatorMotor().setPower(motorPower);
            return;
        }

        if (robotHardware.getElevatorMotor().getCurrentPosition() < 0) {
            if (motorPower > 0f) robotHardware.getElevatorMotor().setPower(motorPower);
            else robotHardware.getElevatorMotor().setPower(0);
            return;
        }

        if (robotHardware.getElevatorMotor().getCurrentPosition() > Constants.MAX_ELEVATOR_TICKS) {
            if (motorPower < 0f) robotHardware.getElevatorMotor().setPower(motorPower);
            else robotHardware.getElevatorMotor().setPower(0f);
            return;
        }

        robotHardware.getElevatorMotor().setPower(motorPower);
        /*craneTelemetry.setValue((motorPower < 0 ? Limb.ELEVATOR_DOWN : Limb.ELEVATOR_UP));
        if (motorPower == 0) craneTelemetry.setValue(Limb.IDLE);
        elevatorMotorTelemetry.setValue("dir %s pow %.2f pos %d", robotHardware.getElevatorMotor().getDirection(), robotHardware.getElevatorMotor().getPower(), robotHardware.getElevatorMotor().getCurrentPosition());
        telemetry.update();*/
    }

    /**
     * Operates the wheel on the back side of the robot with the specified speed
     *
     * @param motorPower Positive value will rotate the robot in left direction,
     *                   while negative value in the right direction
     */
    protected void useWheelMotor(double motorPower) {
        robotHardware.getWheelMotor().setPower(motorPower);

        /*craneTelemetry.setValue(Limb.ROTATING_WHEEL);
        if (motorPower == 0) craneTelemetry.setValue(Limb.IDLE);
        wheelMotorTelemetry.setValue("pow %.2f dir %s", robotHardware.getWheelMotor().getPower(), robotHardware.getWheelMotor().getDirection());
        telemetry.update();*/
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Init Phase
        robotHardware.initTeleOp(hardwareMap);
        useClaws();
        useArm(true);
        waitForStart();
        emergencyButtonCooldown.reset();
        while (opModeIsActive()) {
            drive();
            useLimbs();

            //Emergency elevator button Gamepad2.y
            if (gamepad2.y) {
                if (emergencyButtonCooldown.milliseconds() >= 200) {
                    if (!bypassElevator) {
                        bypassElevator = true;
                    } else {
                        bypassElevator = false;
                        robotHardware.getElevatorMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robotHardware.getElevatorMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    emergencyButtonCooldown.reset();
                }
            }
        }
    }

    /**
     * Drives the robot on the field
     */
    protected abstract void drive();

    /**
     * Operates the limbs of the robot on the field
     */
    protected abstract void useLimbs();
}
