package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.RobotMovementControls;

public class Movements extends RobotMovementControls {
    private static Movements instance = null;

    //prevent instantiation
    private Movements() {

    }

    public static Movements getInstance() {
        if (instance == null) instance = new Movements();
        return instance;
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
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        encodedDriving(motorPower, distance);

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
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
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        encodedDriving(motorPower, distance);

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
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
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        encodedDriving(motorPower, distance);

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
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
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        encodedDriving(motorPower, distance);

        stopMotors();

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
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

    /*FIXME*/
    @Override
    @Deprecated
    public void rotateLeft(float motorPower, float degrees) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.FORWARD);

        float lastAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, globalAngle = 0f;

        do {
            robotHardware.getRightFrontMotor().setPower(motorPower);
            robotHardware.getRightBackMotor().setPower(motorPower);
            robotHardware.getLeftFrontMotor().setPower(motorPower);
            robotHardware.getLeftBackMotor().setPower(motorPower);

            float currentAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            float delta = lastAngle - currentAngle;
            if (delta < -180) delta += 360;
            else if (delta > 180) delta -= 360;

            globalAngle += delta;

            lastAngle = currentAngle;
        } while (globalAngle < degrees);

        stopMotors();
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

    /*FIXME*/
    @Override
    @Deprecated
    public void rotateRight(float motorPower, float degrees) {
        robotHardware.getRightFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftFrontMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getRightBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getLeftBackMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        float lastAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, globalAngle = 0f;

        do {
            robotHardware.getRightFrontMotor().setPower(motorPower);
            robotHardware.getRightBackMotor().setPower(motorPower);
            robotHardware.getLeftFrontMotor().setPower(motorPower);
            robotHardware.getLeftBackMotor().setPower(motorPower);

            float currentAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            float delta = lastAngle - currentAngle;
            if (delta < -180) delta += 360;
            else if (delta > 180) delta -= 360;

            globalAngle += delta;

            lastAngle = currentAngle;
        } while (globalAngle == 0f);

        do {
            robotHardware.getRightFrontMotor().setPower(motorPower);
            robotHardware.getRightBackMotor().setPower(motorPower);
            robotHardware.getLeftFrontMotor().setPower(motorPower);
            robotHardware.getLeftBackMotor().setPower(motorPower);

            float currentAngle = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            float delta = lastAngle - currentAngle;
            if (delta < -180) delta += 360;
            else if (delta > 180) delta -= 360;

            globalAngle += delta;

            lastAngle = currentAngle;
        } while (-globalAngle < degrees);

        stopMotors();
    }

    @Override
    public void stopMotors() {
        robotHardware.getRightFrontMotor().setPower(0f);
        robotHardware.getRightBackMotor().setPower(0f);
        robotHardware.getLeftFrontMotor().setPower(0f);
        robotHardware.getLeftBackMotor().setPower(0f);
    }

    /**
     * Drives the robot using encoders. This method should be used inside an overloaded method designed for a specific direction
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     * @param distance   desired distance in centimetres
     * @see RobotMovementControls#driveForward(float, float)
     * @see RobotMovementControls#driveBackward(float, float)
     * @see RobotMovementControls#driveLeft(float, float)
     * @see RobotMovementControls#driveRight(float, float)
     */
    private void encodedDriving(float motorPower, float distance) {
        robotHardware.setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int drivingTarget = (int) (distance * TICKS_PER_CENTIMETRE);

        robotHardware.getLeftFrontMotor().setTargetPosition(drivingTarget);
        robotHardware.getLeftBackMotor().setTargetPosition(drivingTarget);
        robotHardware.getRightFrontMotor().setTargetPosition(drivingTarget);
        robotHardware.getRightBackMotor().setTargetPosition(drivingTarget);

        robotHardware.getRightFrontMotor().setPower(motorPower);
        robotHardware.getRightBackMotor().setPower(motorPower);
        robotHardware.getLeftFrontMotor().setPower(motorPower);
        robotHardware.getLeftBackMotor().setPower(motorPower);

        robotHardware.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotHardware.getRightFrontMotor().isBusy() &&
                robotHardware.getRightBackMotor().isBusy() &&
                robotHardware.getLeftFrontMotor().isBusy() &&
                robotHardware.getLeftBackMotor().isBusy()
        ) {
            robotHardware.getRightFrontMotor().setPower(motorPower);
            robotHardware.getRightBackMotor().setPower(motorPower);
            robotHardware.getLeftFrontMotor().setPower(motorPower);
            robotHardware.getLeftBackMotor().setPower(motorPower);
        }
    }
}
