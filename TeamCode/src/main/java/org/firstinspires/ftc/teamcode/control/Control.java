package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Controller;

public class Control extends Controller {
    private static Control instance = null;


    /**
     * Get the instance of the Control singleton class
     * @return the instance of the class
     */
    public static Control getInstance() {
        if (instance == null) instance = new Control();
        return instance;
    }

    private Control() {
        super();
    }

    @Override
    public void driveForward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveBackward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveLeft(float motorPower) {
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
    public void driveRight(float motorPower) {
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
    public void driveDiagonallyRightForward(float motorPower) {
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(0f);
    }

    @Override
    public void driveDiagonallyLeftForward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveDiagonallyRightBackward(float motorPower) {
        robotHardware.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.rightFrontMotor.setPower(motorPower);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(motorPower);
    }

    @Override
    public void driveDiagonallyLeftBackward(float motorPower) {
        robotHardware.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(motorPower);
        robotHardware.leftFrontMotor.setPower(motorPower);
        robotHardware.leftBackMotor.setPower(0f);
    }

    @Override
    public void steerLeft(float motorPower) {
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
    public void steerRight(float motorPower) {
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
    public void stopMotors() {
        robotHardware.rightFrontMotor.setPower(0f);
        robotHardware.rightBackMotor.setPower(0f);
        robotHardware.leftFrontMotor.setPower(0f);
        robotHardware.leftBackMotor.setPower(0f);
    }
}
