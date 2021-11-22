package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.util.RobotMovementControls;

public class Movements extends RobotMovementControls {
    private static Movements instance = null;
    //prevent instantiation
    private Movements() {

    }

    /**
     * Get the Movements instance
     * @return the instance of the class
     */
    public static Movements getInstance(){
        if (instance == null) instance = new Movements();
        return instance;
    }

    /**
     * Initializes the Movements object with the hardware map
     * @param hardwareMap never null map responsible for configuring the robot
     */
    public void init(@NonNull HardwareMap hardwareMap){
        robotHardware.init(hardwareMap);
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
