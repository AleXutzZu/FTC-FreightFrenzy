package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Limbs;
import org.firstinspires.ftc.teamcode.control.Movements;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous(name = "Basic Autonomous", group = "Testing Purposes")
public class BasicAutonomous extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final RobotHardware robotHardware = RobotHardware.getInstance();
    private final Limbs robotLimbs = Limbs.getInstance();
    private final Movements robotMovements = Movements.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap);
        while (!isStopRequested() && !robotHardware.getGyroscope().isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("IMU Status", robotHardware.getGyroscope().getCalibrationStatus().toString());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            //SQUARE TO THE LEFT
            robotMovements.driveForward(1f, 100f);
            robotMovements.rotateLeft(1f, 90f);
            robotMovements.driveForward(1f, 100f);
            robotMovements.rotateLeft(1f, 90f);
            robotMovements.driveForward(1f, 100f);
            robotMovements.rotateLeft(1f, 90f);
            robotMovements.driveForward(1f, 100f);

            //SQUARE TO THE RIGHT
            robotMovements.driveForward(1f, 100f);
            robotMovements.rotateRight(1f, 90f);
            robotMovements.driveForward(1f, 100f);
            robotMovements.rotateRight(1f, 90f);
            robotMovements.driveForward(1f, 100f);
            robotMovements.rotateRight(1f, 90f);
            robotMovements.driveForward(1f, 100f);
        }
    }
}
