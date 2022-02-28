package org.firstinspires.ftc.teamcode.debug;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Debug;

public class DrivetrainDebug implements Debug {
    private final RobotHardware robotHardware = RobotHardware.getInstance();

    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    public DrivetrainDebug(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
    }

    private enum Direction {
        FORWARD, BACKWARD,
        STRAFE_LEFT, STRAFE_RIGHT,
        ROTATE_LEFT, ROTATE_RIGHT,
        DIAGONALLY_RIGHT_FORWARD, DIAGONALLY_RIGHT_BACKWARD,
        DIAGONALLY_LEFT_FORWARD, DIAGONALLY_LEFT_BACKWARD,
        IDLE
    }

    @Override
    public void update(@NonNull Telemetry telemetry) {
        double averagePower = (Math.abs(leftFrontMotor.getPower()) + Math.abs(rightFrontMotor.getPower()) + Math.abs(leftBackMotor.getPower()) + Math.abs(rightBackMotor.getPower())) / 4;

        telemetry.addData("Drivetrain", "dir %s pow %.2f",
                getDirection(leftFrontMotor.getPower(), rightFrontMotor.getPower(), leftBackMotor.getPower(), rightBackMotor.getPower()),
                averagePower)
                .addData("Left Front Motor", "pow %.2f pos %d", leftFrontMotor.getPower(), leftFrontMotor.getCurrentPosition())
                .addData("Right Front Motor", "pow %.2f pos %d", rightFrontMotor.getPower(), rightFrontMotor.getCurrentPosition())
                .addData("Left Back Motor", "pow %.2f pos %d", leftBackMotor.getPower(), leftBackMotor.getCurrentPosition())
                .addData("Right Back Motor", "pow %.2f pos %d", rightBackMotor.getPower(), rightBackMotor.getCurrentPosition());
        telemetry.update();
    }

    private Direction getDirection(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        if (leftFrontPower == 0 && rightFrontPower == 0 && leftBackPower == 0 && rightBackPower == 0) {
            return Direction.IDLE;
        }
        if (leftFrontPower > 0 && rightFrontPower > 0 && leftBackPower > 0 && rightBackPower > 0) {
            return Direction.FORWARD;
        }
        if (leftFrontPower < 0 && rightFrontPower < 0 && leftBackPower < 0 && rightBackPower < 0) {
            return Direction.BACKWARD;
        }
        if (leftFrontPower < 0 && rightFrontPower > 0 && leftBackPower > 0 && rightBackPower < 0) {
            return Direction.STRAFE_LEFT;
        }
        if (leftFrontPower > 0 && rightFrontPower < 0 && leftBackPower < 0 && rightBackPower > 0) {
            return Direction.STRAFE_RIGHT;
        }
        if (leftFrontPower < 0 && rightFrontPower > 0 && leftBackPower < 0 && rightBackPower > 0) {
            return Direction.ROTATE_LEFT;
        }
        if (leftFrontPower > 0 && rightFrontPower < 0 && leftBackPower > 0 && rightBackPower < 0) {
            return Direction.ROTATE_RIGHT;
        }
        if (leftFrontPower == 0 && rightFrontPower > 0 && leftBackPower > 0 && rightBackPower == 0) {
            return Direction.DIAGONALLY_RIGHT_FORWARD;
        }
        if (leftFrontPower < 0 && rightFrontPower == 0 && leftBackPower == 0 && rightBackPower < 0) {
            return Direction.DIAGONALLY_RIGHT_BACKWARD;
        }
        if (leftFrontPower > 0 && rightFrontPower == 0 && leftBackPower == 0 && rightBackPower > 0) {
            return Direction.DIAGONALLY_LEFT_FORWARD;
        }
        if (leftFrontPower == 0 && rightFrontPower < 0 && leftBackPower < 0 && rightBackPower == 0) {
            return Direction.DIAGONALLY_LEFT_BACKWARD;
        }
        return Direction.IDLE;
    }
}
