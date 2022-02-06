package org.firstinspires.ftc.teamcode.opmodes.teleop.debugging;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.control.GamepadMappings;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Gyroscope Debugging", group = "Debugging")
@Disabled
public class GyroDebugging extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final RobotHardware robotHardware = RobotHardware.getInstance();
    private GamepadMappings robotControl;

    @Override
    public void init() {
        robotHardware.initMotors(hardwareMap);
        robotControl = new GamepadMappings(gamepad1, gamepad2, DcMotorSimple.Direction.FORWARD);
        robotControl.getRobotMovements().stopMotors();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        robotControl.drive();
        float z = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        float x = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        float y = robotHardware.getGyroscope().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;

        telemetry.addData("Gyroscope", "X %.2f Y %.2f Z %.2f", x, y, z);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void stop() {

    }
}
