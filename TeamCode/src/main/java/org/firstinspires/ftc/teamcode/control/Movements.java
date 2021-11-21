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
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void driveForward(float motorPower, float distance) {

    }

    @Override
    public void driveBackward(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void driveBackward(float motorPower, float distance) {

    }

    @Override
    public void driveLeft(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void driveLeft(float motorPower, float distance) {

    }

    @Override
    public void driveRight(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void driveRight(float motorPower, float distance) {

    }

    @Override
    public void driveDiagonallyRightForward(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(0f);
        robotHardware.getLeftFrontMotor().setPower(0f);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void driveDiagonallyLeftForward(float motorPower) {
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.getRightFrontMotor().setPower(0f);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(0f);
    }

    @Override
    public void driveDiagonallyRightBackward(float motorPower) {
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.getRightFrontMotor().setPower(0f);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(0f);
    }

    @Override
    public void driveDiagonallyLeftBackward(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(0f);
        robotHardware.getLeftFrontMotor().setPower(0f);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void steerForward(float leftMotorsPower, float rightMotorsPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.getRightFrontMotor().setPower(rightMotorsPower);
        robotHardware.getRightBackMotor().setPower(rightMotorsPower);
        robotHardware.getLeftFrontMotor().setPower(leftMotorsPower);
        robotHardware.getLeftBackMotor().setPower(leftMotorsPower);
    }

    @Override
    public void steerBackward(float leftMotorsPower, float rightMotorsPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.getRightFrontMotor().setPower(rightMotorsPower);
        robotHardware.getRightBackMotor().setPower(rightMotorsPower);
        robotHardware.getLeftFrontMotor().setPower(leftMotorsPower);
        robotHardware.getLeftBackMotor().setPower(leftMotorsPower);
    }

    @Override
    public void rotateLeft(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void rotateRight(float motorPower) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);
    }

    @Override
    public void stopMotors() {
        robotHardware.getRightFrontMotor().setPower(0f);
        robotHardware.getRightBackMotor().setPower(0f);
        robotHardware.getLeftFrontMotor().setPower(0f);
        robotHardware.getLeftBackMotor().setPower(0f);
    }
}
