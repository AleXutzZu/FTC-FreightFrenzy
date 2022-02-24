package org.firstinspires.ftc.teamcode.opmodes.teleop.movement;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.TeleOpControl;

@TeleOp(name = "Proficient Omni Movement", group = "Testing Purposes")
public class ProficientOmniMovement extends TeleOpControl {
    /**
     * Cooldown for opening/closing claws
     */
    private final ElapsedTime clawButtonCooldown = new ElapsedTime();

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
        /*
        Key mappings
        Right Trigger - Rotate wheel to the right
        Left Trigger - Rotate wheel to the left
        Right joystick - bring up/down the elevator
        dpad up/down - put up/down the arm
        LB - open/close claws
         */
        //Wheel rotation
        float wheelPower = -gamepad2.right_trigger + gamepad2.left_trigger;
        if (wheelPower != 0) {
            useWheelMotor(wheelPower);
            return;
        } else useWheelMotor(0);

        //Elevator controls
        float elevatorY = -gamepad2.right_stick_y;
        boolean isRightJoyStickActive = (elevatorY != 0);
        if (isRightJoyStickActive) {
            useElevator(elevatorY);
            return;
        } else useElevator(0);

        //Arm controls
        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                useArm(true);
                return;
            }
            if (!gamepad2.dpad_up) {
                useArm(false);
                return;
            }
        }

        //Claw control
        if (gamepad2.b) {
            if (clawButtonCooldown.time() >= 0.5) {
                clawButtonCooldown.reset();
                useClaws();
            }
        }
    }
}
