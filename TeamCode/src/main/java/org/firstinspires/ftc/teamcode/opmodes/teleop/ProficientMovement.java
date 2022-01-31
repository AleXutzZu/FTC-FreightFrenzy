package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.GamepadMappings;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.LimbPosition;


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
        telemetry.clear();
        robotControl.getTimer().reset();
    }

    @Override
    public void loop() {
        Direction direction = robotControl.drive();
        LimbPosition limb = robotControl.useLimbs();
        if (robotControl.isDebug()) {
            telemetry.addData("Motor State", "Direction -> %s Left Front: %.2f Right Front: %.2f Left Back: %.2f Right Back: %.2f",
                    direction.toString(),
                    robotControl.getLeftFrontMotorPower(),
                    robotControl.getRightFrontMotorPower(),
                    robotControl.getLeftBackMotorPower(),
                    robotControl.getRightBackMotorPower())
                    .addData("Limb State", "Direction -> %s", limb);
        }
    }

    @Override
    public void stop() {
        robotControl.getRobotMovements().stopMotors();
    }
}
