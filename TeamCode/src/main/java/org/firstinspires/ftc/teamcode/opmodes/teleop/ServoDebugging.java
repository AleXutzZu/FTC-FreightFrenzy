package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(group = "Testing Purposes", name = "Servo Debugging")
@Disabled
public class ServoDebugging extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robotHardware;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robotHardware = RobotHardware.getInstance();
        robotHardware.init(hardwareMap);
        robotHardware.getArmBase().setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
        robotHardware.getArmBase().setPosition(0.5f);
    }

    @Override
    public void loop() {
        robotHardware.getArmBase().setDirection(Servo.Direction.FORWARD);
        robotHardware.getArmBase().setPosition(1f);
        telemetry.addData("Arm_base Servo", "Position %.2f", robotHardware.getArmBase().getPosition());
    }

    @Override
    public void stop() {

    }
}
