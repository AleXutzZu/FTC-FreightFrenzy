package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotMovementControls;

public class Movements extends RobotMovementControls {

    private Movements() {

    }

    public static class Builder {
        private final @NonNull HardwareMap hardwareMap;

        /**
         * Creates a builder instance for the Movements class
         *
         * @param hardwareMap Object responsible for mapping the physical names to their virtual counterparts
         * @see org.firstinspires.ftc.teamcode.hardware.RobotHardware
         */
        public Builder(@NonNull HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        /**
         * Builds the Movements object
         *
         * @return a Movements object with the desired hardware map
         * @see Movements
         */
        public Movements build() {
            Movements movements = new Movements();
            movements.robotHardware.init(hardwareMap);
            return movements;
        }
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
