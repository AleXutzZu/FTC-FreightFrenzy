package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.TeleOpControl;

@TeleOp(name = "Motor Tick Test", group = "Testing Purposes")
public class TeleOpControlImpl extends TeleOpControl {

    private final ElapsedTime buttonTimer = new ElapsedTime();
    private boolean motorDirection = false;

    @Override
    public void init() {
        robotHardware.initTeleOp(hardwareMap);
    }

    @Override
    public void start() {
        buttonTimer.reset();
    }

    @Override
    protected void drive() {
        if (gamepad1.y) {
            if (buttonTimer.time() > 1f) {
                buttonTimer.reset();
                if (motorDirection) {
                    motorDirection = false;
                    robotHardware.getWheelMotor().setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    motorDirection = true;
                    robotHardware.getWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
        }
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        robotHardware.getWheelMotor().setPower(power);

        telemetry.addData("Wheel Info", "Direction %s, ticks %d, pow %.2f", robotHardware.getWheelMotor().getDirection(), robotHardware.getWheelMotor().getCurrentPosition(), power);
    }

    @Override
    protected void useLimbs() {
    }
}
