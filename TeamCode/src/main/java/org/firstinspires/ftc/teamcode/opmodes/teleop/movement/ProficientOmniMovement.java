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
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double targetLeftFrontPower  = axial + lateral + yaw;
        double targetRightFrontPower = axial - lateral - yaw;
        double targetLeftBackPower   = axial - lateral + yaw;
        double targetRightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(targetLeftFrontPower), Math.abs(targetRightFrontPower));
        max = Math.max(max, Math.abs(targetLeftBackPower));
        max = Math.max(max, Math.abs(targetRightBackPower));

        if (max > 1.0) {
            targetLeftFrontPower  /= max;
            targetRightFrontPower /= max;
            targetLeftBackPower   /= max;
            targetRightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            targetLeftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            targetLeftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            targetRightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            targetRightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

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
