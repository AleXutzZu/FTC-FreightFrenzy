package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Constants;

public abstract class AutonomousControl extends LinearOpMode {
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
     * Elevator distances for each level in centimetres
     */
    private enum ElevatorLevels {
        START(0), LEVEL_ONE(0), LEVEL_TWO(0), LEVEL_THREE(0), MAX(0);

        ElevatorLevels(double distance) {

            this.distance = distance;
        }

        private final double distance;

        public double getDistance() {
            return distance;
        }
    }

    /**
     * Drives the robot in a straight direction for the desired distance with the desired power
     *
     * @param distance value representing the desired distance in centimetres. If negative, the robot will go backwards, forwards otherwise
     */
    protected void driveStraight(double distance) {
        drive(distance, (distance < 0d ? DrivingDirection.BACKWARD : DrivingDirection.FORWARD));
    }

    /**
     * Drives the robot on a horizontal direction for the desired distance with the desired power
     *
     * @param distance positive value representing the desired distance in centimetres.
     */
    protected void driveSideways(double distance) {
        drive(distance, (distance < 0f ? DrivingDirection.STRAFE_LEFT : DrivingDirection.STRAFE_RIGHT));
    }

    /**
     * Rotates the robot around its central axis with the desired power for the desired degrees
     *
     * @param degrees Value between -180 and 180 (inclusive) which the robot should rotate to
     */
    protected void rotate(double degrees) {
        degrees = Range.clip(degrees, -180, 180);
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        if (degrees > 0f) {
            rightFrontPower = Constants.ROTATION_POWER;
            leftFrontPower = -Constants.ROTATION_POWER;
            rightBackPower = Constants.ROTATION_POWER;
            leftBackPower = -Constants.ROTATION_POWER;
        } else {
            rightFrontPower = -Constants.ROTATION_POWER;
            leftFrontPower = Constants.ROTATION_POWER;
            rightBackPower = -Constants.ROTATION_POWER;
            leftBackPower = Constants.ROTATION_POWER;
        }

        double lastAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        double relativeAngle = 0f;
        /*
        TODO
            - Experiment with lower speeds
         */
        double yaw = degrees;

        while (opModeIsActive() && Math.abs(yaw) > 14f) {
            robotHardware.getRightFrontMotor().setPower(rightFrontPower);
            robotHardware.getRightBackMotor().setPower(rightBackPower);
            robotHardware.getLeftFrontMotor().setPower(leftFrontPower);
            robotHardware.getLeftBackMotor().setPower(leftBackPower);

            double currentAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

            double delta = currentAngle - lastAngle;

            if (delta > 180) delta -= 360;
            else if (delta <= -180) delta += 360;

            relativeAngle += delta;
            lastAngle = currentAngle;
            yaw = degrees - relativeAngle;
        }
        stopMotors();
        sleep(100);
    }

    /**
     * Rotates the robot to the absolute angle (based on the current orientation of the robot)
     *
     * @param target value representing the target to turn to
     */
    protected void rotateTo(double target) {
        target = Range.clip(target, -180, 180);
        double currentAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        double yaw = target - currentAngle;

        if (yaw > 180) yaw -= 360;
        else if (yaw < -180) yaw += 360;

        rotate(yaw);
    }

    /**
     * Completely brings the motors to a halt.
     */
    protected void stopMotors() {
        if (opModeIsActive()) {
            robotHardware.getRightFrontMotor().setPower(0);
            robotHardware.getRightBackMotor().setPower(0);
            robotHardware.getLeftFrontMotor().setPower(0);
            robotHardware.getLeftBackMotor().setPower(0);
        }
    }

    private void drive(double distance, @NonNull DrivingDirection direction) {
        distance = Math.abs(distance);

        int drivingTarget = (int) (distance * Constants.DRIVING_TICKS_PER_CENTIMETRE);

        switch (direction) {

            case FORWARD:
                robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case BACKWARD:
                robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case STRAFE_LEFT:
                robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case STRAFE_RIGHT:
                robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                break;
        }

        robotHardware.getRightBackMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getRightFrontMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getLeftFrontMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getLeftBackMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotHardware.getLeftFrontMotor().setTargetPosition(drivingTarget);
        robotHardware.getLeftBackMotor().setTargetPosition(drivingTarget);
        robotHardware.getRightFrontMotor().setTargetPosition(drivingTarget);
        robotHardware.getRightBackMotor().setTargetPosition(drivingTarget);

        robotHardware.getRightFrontMotor().setPower(Constants.DRIVING_POWER);
        robotHardware.getRightBackMotor().setPower(Constants.DRIVING_POWER);
        robotHardware.getLeftFrontMotor().setPower(Constants.DRIVING_POWER);
        robotHardware.getLeftBackMotor().setPower(Constants.DRIVING_POWER);

        robotHardware.getRightBackMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getRightFrontMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getLeftFrontMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getLeftBackMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (
                robotHardware.getRightFrontMotor().isBusy() &&
                        robotHardware.getRightBackMotor().isBusy() &&
                        robotHardware.getLeftFrontMotor().isBusy() &&
                        robotHardware.getLeftBackMotor().isBusy())
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

            robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
            robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
            robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        }
        sleep(100);
    }

    /**
     * Operates the arm on the front of the robot. It uses the ARM_UP position
     * the arm up, and ARM_DOWN to position it down
     *
     * @param armUp true if the arm should come up, false if it should come down
     * @see Constants#ARM_UP
     * @see Constants#ARM_DOWN
     */
    protected void useArm(boolean armUp) {
        if (!isStopRequested()) {
            if (armUp) robotHardware.getArmBase().setPosition(Constants.ARM_UP);
            else robotHardware.getArmBase().setPosition(Constants.ARM_DOWN);
        }
        sleep(100);
    }

    /**
     * Operates the claws on the robot arm. It uses the CLAWS_CLOSED position to close
     * the claws, and CLAWS_OPENED to open them
     *
     * @param closeClaws true if the claws should close, false if they should open
     * @see Constants#CLAWS_CLOSED
     * @see Constants#CLAWS_OPENED
     */
    protected void useClaws(boolean closeClaws) {
        if (!isStopRequested()) {
            if (closeClaws) {
                robotHardware.getRightClaw().setPosition(Constants.CLAWS_CLOSED);
                robotHardware.getLeftClaw().setPosition(Constants.CLAWS_CLOSED);
            } else {
                robotHardware.getLeftClaw().setPosition(Constants.CLAWS_OPENED);
                robotHardware.getRightClaw().setPosition(Constants.CLAWS_OPENED);
            }
        }
        sleep(100);
    }

    //TODO

    /**
     * Operates the wheel on the back of the robot
     */
    protected void useWheel() {
        if (opModeIsActive()) {
            robotHardware.getWheelMotor().setPower(1);
        }
        sleep(100);
    }

    protected void useElevator(int level) {
        int targetPos = 0;
        switch (level) {
            case 1:
                targetPos = 1600;
                break;
            case 2:
                targetPos = 4100;
                break;
            case 3:
                targetPos = 6300;
        }
        robotHardware.getElevatorMotor().setTargetPosition(targetPos);
        robotHardware.getElevatorMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.getElevatorMotor().setPower(1);
        while (robotHardware.getElevatorMotor().isBusy()) ;

        robotHardware.getElevatorMotor().setPower(0);
        robotHardware.getElevatorMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * This function will identify where the team element is located and return the
     * elevator level necessary
     *
     * @return the elevator level corresponding to the position, level 1 if no position
     * is found
     */
    protected int identifyElement() {
        if (robotHardware.getLeftDistanceSensor().getDistance(DistanceUnit.CM) < 30) {
            return 2;
        }

        driveStraight(15);

        if (robotHardware.getLeftDistanceSensor().getDistance(DistanceUnit.CM) < 30) {
            return 3;
        }

        return 1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initAutonomous(hardwareMap);
        useClaws(true);
        useArm(true);
        waitForStart();
        run();
    }

    protected abstract void run();
}
