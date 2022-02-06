package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class AutonomousControl extends LinearOpMode {
    /*
    TODO
        - Possibly update values to match goBILDA motors and wheels.
        - Currently using TETRIX Motors and HD Mecanum Wheels
     */
    /**
     * <p>Ticks used for encoder driving in the overloaded methods with distance parameter</p>
     * <a href="https://asset.pitsco.com/sharedimages/resources/torquenado_dcmotorspecs.pdf">TETRIX Motor Documentation</a>
     */
    protected static final int MOTOR_TICK_RATE = 1440;

    /**
     * <p>Wheel diameter in <b>centimetres</b></p>
     * <a href= "https://www.andymark.com/products/4-in-hd-mecanum-wheel-set-options">HD Mecanum Wheels</a>
     */
    protected static final float WHEEL_DIAMETER = 10.16f;

    /**
     * <p>Ticks per centimetre based on motor and wheel specs</p>
     * <p>Defined as <b>MOTOR_TICK_RATE / WHEEL_DIAMETER</b></p>
     *
     * @see AutonomousControl#WHEEL_DIAMETER
     * @see AutonomousControl#MOTOR_TICK_RATE
     */
    protected static final float TICKS_PER_CENTIMETRE = MOTOR_TICK_RATE / (WHEEL_DIAMETER * (float) Math.PI);

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
     * Directions a robot may take while driving straight or sideways
     */
    private enum DrivingDirection {
        FORWARD, BACKWARD, STRAFE_LEFT, STRAFE_RIGHT
    }

    /**
     * Drives the robot in a straight direction for the desired distance with the desired power
     *
     * @param motorPower power that should be given to the motors. Negative power will result in backwards movement, while positive will be forwards.
     *                   The value should range from -1.0 to 1.0
     * @param distance   positive value representing the desired distance in centimetres.
     * @throws IllegalArgumentException if the distance is negative
     */
    protected void driveStraight(float motorPower, float distance) throws IllegalArgumentException {
        if (distance < 0f) {
            throw new IllegalArgumentException("Expected a positive value for distance parameter");
        }
        drive(motorPower, distance, (motorPower < 0f ? DrivingDirection.BACKWARD : DrivingDirection.FORWARD));
    }

    /**
     * Drives the robot on a horizontal direction for the desired distance with the desired power
     *
     * @param motorPower power that should be given to the motors. Negative power will result in left movement, while positive will be right movement.
     *                   The value should range from -1.0 to 1.0
     * @param distance   positive value representing the desired distance in centimetres.
     * @throws IllegalArgumentException if the distance is negative
     */
    protected void driveSideways(float motorPower, float distance) throws IllegalArgumentException {
        if (distance < 0f) {
            throw new IllegalArgumentException("Expected a positive value for distance parameter");
        }
        drive(motorPower, distance, (motorPower < 0f ? DrivingDirection.STRAFE_LEFT : DrivingDirection.STRAFE_RIGHT));
    }

    /**
     * Rotates the robot around its central axis with the desired power for the desired degrees
     *
     * @param motorPower power that should be given to the motors. Positive power will result in trigonometric sense, while negative power will be counter-trigonometric sense.
     * @param degrees    Value between 0 and 180 (inclusive) which the robot should rotate to
     * @throws IllegalArgumentException if degrees < 0 or degrees > 180
     */
    protected void rotate(float motorPower, float degrees) throws IllegalArgumentException {
        if (degrees < 0f || degrees > 180f) {
            throw new IllegalArgumentException("Expected degrees between 0 and 180");
        }
        float leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        if (motorPower >= 0f) {
            rightFrontPower = -motorPower;
            leftFrontPower = motorPower;
            rightBackPower = -motorPower;
            leftBackPower = motorPower;
        } else {
            rightFrontPower = motorPower;
            leftFrontPower = -motorPower;
            rightBackPower = motorPower;
            leftBackPower = -motorPower;
        }

        float lastAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        float relativeAngle = 0f;
        /*
        TODO
            - Implement error check
            - Experiment with lower speeds
         */
        while (opModeIsActive() && Math.abs(relativeAngle) < degrees) {
            robotHardware.getRightFrontMotor().setPower(rightFrontPower);
            robotHardware.getRightBackMotor().setPower(rightBackPower);
            robotHardware.getLeftFrontMotor().setPower(leftFrontPower);
            robotHardware.getLeftBackMotor().setPower(leftBackPower);

            float currentAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

            float delta = currentAngle - lastAngle;

            if (delta > 180) delta -= 360;
            else if (delta <= -180) delta += 360;

            relativeAngle += delta;
            lastAngle = currentAngle;
        }
        stopMotors();
    }

    /**
     * Completely brings the motors to a halt.
     */
    protected void stopMotors() {
        if (opModeIsActive()) {
            robotHardware.getRightFrontMotor().setPower(0f);
            robotHardware.getRightBackMotor().setPower(0f);
            robotHardware.getLeftFrontMotor().setPower(0f);
            robotHardware.getLeftBackMotor().setPower(0f);
        }
    }

    private void drive(float motorPower, float distance, @NonNull DrivingDirection direction) {
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
        }

        robotHardware.getRightBackMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getRightFrontMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getLeftFrontMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getLeftBackMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int drivingTarget = (int) (distance * TICKS_PER_CENTIMETRE);

        robotHardware.getLeftFrontMotor().setTargetPosition(drivingTarget);
        robotHardware.getLeftBackMotor().setTargetPosition(drivingTarget);
        robotHardware.getRightFrontMotor().setTargetPosition(drivingTarget);
        robotHardware.getRightBackMotor().setTargetPosition(drivingTarget);

        robotHardware.getRightFrontMotor().setPower(rightFrontPower);
        robotHardware.getRightBackMotor().setPower(rightBackPower);
        robotHardware.getLeftFrontMotor().setPower(leftFrontPower);
        robotHardware.getLeftBackMotor().setPower(leftBackPower);

        robotHardware.getRightBackMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getRightFrontMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getLeftFrontMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getLeftBackMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                robotHardware.getRightFrontMotor().isBusy() &&
                robotHardware.getRightBackMotor().isBusy() &&
                robotHardware.getLeftFrontMotor().isBusy() &&
                robotHardware.getLeftBackMotor().isBusy()
        ) {
            /*
            TODO
                - Use the sensors to prevent collision
             */
        }
        stopMotors();

        if (opModeIsActive()) {
            robotHardware.getRightBackMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotHardware.getRightFrontMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotHardware.getLeftFrontMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotHardware.getLeftBackMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Operates the arm on the front of the robot. It uses the ARM_UP position
     * the arm up, and ARM_DOWN to position it down
     *
     * @param armUp true if the arm should come up, false if it should come down
     * @see AutonomousControl#ARM_UP
     * @see AutonomousControl#ARM_DOWN
     */
    protected void useArm(boolean armUp) {
        if ((opModeIsActive() || isStarted()) && !isStopRequested()) {
            if (armUp) robotHardware.getArmBase().setPosition(ARM_UP);
            else robotHardware.getArmBase().setPosition(ARM_DOWN);
        }
    }

    /**
     * Operates the claws on the robot arm. It uses the CLAWS_CLOSED position to close
     * the claws, and CLAWS_OPENED to open them
     *
     * @param closeClaws true if the claws should close, false if they should open
     * @see AutonomousControl#CLAWS_CLOSED
     * @see AutonomousControl#CLAWS_OPENED
     */
    protected void useClaws(boolean closeClaws) {
        if (opModeIsActive()) {
            if (closeClaws) {
                robotHardware.getRightClaw().setPosition(CLAWS_CLOSED);
                robotHardware.getLeftClaw().setPosition(CLAWS_CLOSED);
            } else {
                robotHardware.getLeftClaw().setPosition(CLAWS_OPENED);
                robotHardware.getRightClaw().setPosition(CLAWS_OPENED);
            }
        }
    }

    //TODO

    /**
     * Operates the wheel on the back of the robot
     */
    protected void useWheel() {
        if (opModeIsActive()) {
            robotHardware.getWheelMotor().setPower(1f);
        }
    }

    //TODO
    protected void useElevator(int level) {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initMotors(hardwareMap);
        robotHardware.initGyro(hardwareMap);
        robotHardware.initSensors(hardwareMap);
        useClaws(true);
        useArm(true);
        waitForStart();
        run();
    }

    protected abstract void run();
}
