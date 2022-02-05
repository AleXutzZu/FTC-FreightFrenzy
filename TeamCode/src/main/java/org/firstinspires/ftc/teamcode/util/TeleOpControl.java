package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class TeleOpControl extends OpMode {
    protected final RobotHardware robotHardware = RobotHardware.getInstance();

    private enum Direction {
        FORWARD, BACKWARD,
        STRAFE_LEFT, STRAFE_RIGHT,
        ROTATE_LEFT, ROTATE_RIGHT,
        DIAGONALLY_RIGHT_FORWARD, DIAGONALLY_RIGHT_BACKWARD,
        DIAGONALLY_LEFT_FORWARD, DIAGONALLY_LEFT_BACKWARD
    }

    protected void driveStraight(float motorPower) {

    }

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
                break;
            case DIAGONALLY_RIGHT_BACKWARD:
                break;
            case DIAGONALLY_LEFT_FORWARD:
                break;
            case DIAGONALLY_LEFT_BACKWARD:
                break;
        }
    }

    @Override
    public void init() {
        robotHardware.init(hardwareMap);
    }
}
