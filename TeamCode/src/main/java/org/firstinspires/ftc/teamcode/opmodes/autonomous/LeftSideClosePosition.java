package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Limbs;
import org.firstinspires.ftc.teamcode.control.Movements;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous(name = "Demo: Left Side Close Position", group = "Demo Cluj-Napoca")
public class LeftSideClosePosition extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final RobotHardware robotHardware = RobotHardware.getInstance();
    private final Limbs robotLimbs = Limbs.getInstance();
    private final Movements robotMovements = Movements.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap);
        robotLimbs.useArm(true);
        waitForStart();

        if (opModeIsActive()) {
            robotMovements.driveForward(1f, 32);
            robotMovements.rotateLeft(1f);
            sleep(1000);
            robotMovements.driveForward(1f, 100);
        }
    }
}
