package org.firstinspires.ftc.teamcode.opmodes.teleop.movement;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.TeleOpControl;

@TeleOp(name = "Proficient Omni Movement", group = "Testing Purposes")
public class ProficientOmniMovement extends TeleOpControl {
    @Override
    protected void drive() {
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sine = Math.sin(theta - Math.PI / 4);
        double cosine = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sine), Math.abs(cosine));

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double targetLeftFrontPower = power * cosine / max + turn;
        double targetRightFrontPower = power * sine / max - turn;
        double targetLeftBackPower = power * sine / max + turn;
        double targetRightBackPower = power * cosine / max - turn;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        if (power + Math.abs(turn) > 1) {
            targetLeftFrontPower /= power + turn;
            targetRightFrontPower /= power + turn;
            targetLeftBackPower /= power + turn;
            targetRightBackPower /= power + turn;
        }

        // Send calculated power to wheels
        robotHardware.getLeftFrontMotor().setPower(targetLeftFrontPower);
        robotHardware.getRightFrontMotor().setPower(targetRightFrontPower);
        robotHardware.getLeftBackMotor().setPower(targetLeftBackPower);
        robotHardware.getRightBackMotor().setPower(targetRightBackPower);
    }

    @Override
    protected void useLimbs() {

    }
}
