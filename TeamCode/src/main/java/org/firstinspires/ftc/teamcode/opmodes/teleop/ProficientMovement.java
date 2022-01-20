package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.GamepadMappings;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Direction;


@TeleOp(name = "Proficient Movement", group = "Testing Purposes")
public class ProficientMovement extends OpMode {
    private GamepadMappings robotControl;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        /*
        Initialize hardware
         */
        RobotHardware robotHardware = RobotHardware.getInstance();
        robotHardware.init(hardwareMap);

        robotControl = new GamepadMappings(gamepad1, gamepad2);
        robotControl.getRobotMovements().stopMotors();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        robotControl.getRobotMovements().stopMotors();
    }

    @Override
    public void loop() {
        Direction direction = robotControl.drive();
        if (robotControl.isDebug()) {
            telemetry.addData("State", "Direction -> %s leftMotors: 0 rightMotors: 0", direction.toString());
        }
        String limb = robotControl.useLimbs();
        /*telemetry.addData("Limb %s",limb);*/
    }

    @Override
    public void stop() {
        robotControl.getRobotMovements().stopMotors();
    }
}
