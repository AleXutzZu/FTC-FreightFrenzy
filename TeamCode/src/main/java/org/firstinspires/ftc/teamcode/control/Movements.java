package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotMovementControls;

public class Movements extends RobotMovementControls {
    /**
     * Creates an instance for control with the specified hardware map
     *
     * @param hardwareMap Object responsible for mapping the physical names to their virtual counterparts
     */
    public Movements(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void driveForward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveForward(float motorPower, float distance) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int drivingTarget = (int) (distance * TICKS_PER_CENTIMETRE);

        robotHardware.leftFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.leftBackMotor.setTargetPosition(drivingTarget);
        robotHardware.rightFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.rightBackMotor.setTargetPosition(drivingTarget);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotHardware.rightFrontMotor.isBusy() &&
                robotHardware.rightBackMotor.isBusy() &&
                robotHardware.leftFrontMotor.isBusy() &&
                robotHardware.leftBackMotor.isBusy()
        ) {
            robotHardware.rightFrontMotor.setPower(motorPower);
            robotHardware.rightBackMotor.setPower(motorPower);
            robotHardware.leftFrontMotor.setPower(motorPower);
            robotHardware.leftBackMotor.setPower(motorPower);
        }

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driveBackward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveBackward(float motorPower, float distance) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int drivingTarget = (int) (distance * TICKS_PER_CENTIMETRE);

        robotHardware.leftFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.leftBackMotor.setTargetPosition(drivingTarget);
        robotHardware.rightFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.rightBackMotor.setTargetPosition(drivingTarget);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotHardware.rightFrontMotor.isBusy() &&
                robotHardware.rightBackMotor.isBusy() &&
                robotHardware.leftFrontMotor.isBusy() &&
                robotHardware.leftBackMotor.isBusy()
        ) {
            robotHardware.rightFrontMotor.setPower(motorPower);
            robotHardware.rightBackMotor.setPower(motorPower);
            robotHardware.leftFrontMotor.setPower(motorPower);
            robotHardware.leftBackMotor.setPower(motorPower);
        }

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driveLeft(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveLeft(float motorPower, float distance) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int drivingTarget = (int) (distance * TICKS_PER_CENTIMETRE);

        robotHardware.leftFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.leftBackMotor.setTargetPosition(drivingTarget);
        robotHardware.rightFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.rightBackMotor.setTargetPosition(drivingTarget);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotHardware.rightFrontMotor.isBusy() &&
                robotHardware.rightBackMotor.isBusy() &&
                robotHardware.leftFrontMotor.isBusy() &&
                robotHardware.leftBackMotor.isBusy()
        ) {
            robotHardware.rightFrontMotor.setPower(motorPower);
            robotHardware.rightBackMotor.setPower(motorPower);
            robotHardware.leftFrontMotor.setPower(motorPower);
            robotHardware.leftBackMotor.setPower(motorPower);
        }

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driveRight(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveRight(float motorPower, float distance) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int drivingTarget = (int) (distance * TICKS_PER_CENTIMETRE);

        robotHardware.leftFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.leftBackMotor.setTargetPosition(drivingTarget);
        robotHardware.rightFrontMotor.setTargetPosition(drivingTarget);
        robotHardware.rightBackMotor.setTargetPosition(drivingTarget);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotHardware.rightFrontMotor.isBusy() &&
                robotHardware.rightBackMotor.isBusy() &&
                robotHardware.leftFrontMotor.isBusy() &&
                robotHardware.leftBackMotor.isBusy()
        ) {
            robotHardware.rightFrontMotor.setPower(motorPower);
            robotHardware.rightBackMotor.setPower(motorPower);
            robotHardware.leftFrontMotor.setPower(motorPower);
            robotHardware.leftBackMotor.setPower(motorPower);
        }

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void driveDiagonallyRightForward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveDiagonallyLeftForward(float motorPower) {
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(0f);
    }

    @Override
    public void driveDiagonallyRightBackward(float motorPower) {
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(0f);
    }

    @Override
    public void driveDiagonallyLeftBackward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void steerForward(float leftMotorsPower, float rightMotorsPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(rightMotorsPower);
        robotHardware.rightBackMotor.setPower(rightMotorsPower);
        robotHardware.leftFrontMotor.setPower(leftMotorsPower);
        robotHardware.leftBackMotor.setPower(leftMotorsPower);
    }

    @Override
    public void steerBackward(float leftMotorsPower, float rightMotorsPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(rightMotorsPower);
        robotHardware.rightBackMotor.setPower(rightMotorsPower);
        robotHardware.leftFrontMotor.setPower(leftMotorsPower);
        robotHardware.leftBackMotor.setPower(leftMotorsPower);
    }

    @Override
    public void rotateLeft(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void rotateRight(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void stopMotors() {
        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(0f);
    }
}
