package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.AutonomousUtil;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.logging.Level;

@Autonomous(name = "Experimental Finite State Machine", group = "Testing Purposes")
public class ExperimentalFSM extends AutonomousUtil {
    private enum MachineState {
        START,
        GO_TO_CAROUSEL,
        SPIN_DUCK,
        GO_TO_BARCODE, CHECK_BARCODE_1, CHECK_BARCODE_2,
        GO_TO_SHIPPING_HUB, LIFT_FREIGHT, DUMP_FREIGHT,
        GO_TO_STORAGE_UNIT, IDENTIFY_FREIGHT, POSITION_TO_PICK_FREIGHT, PICK_FREIGHT,
        IDLE
    }

    private final Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime spinTimer = new ElapsedTime();
    private static final double spinTime = 3.0d;
    private MachineState state = MachineState.START;
    private ElevatorLevel level = ElevatorLevel.LEVEL_THREE;

    @Override
    protected void run() {

        Trajectory carouselTrajectory = drive.trajectoryBuilder(startPose).lineToLinearHeading(new Pose2d(-55, -49, Math.toRadians(45))).build();
        Trajectory barcodeTrajectory = drive.trajectoryBuilder(carouselTrajectory.end()).lineToLinearHeading(new Pose2d(-45, -48, Math.toRadians(0))).build();
        Trajectory barcode2Trajectory = drive.trajectoryBuilder(barcodeTrajectory.end()).lineTo(new Vector2d(-35, -48)).build();

        boolean checkedBarcode = false;
        while (opModeIsActive()) {
            switch (state) {
                case START:
                    drive.setPoseEstimate(startPose);
                    state = MachineState.GO_TO_CAROUSEL;
                    drive.followTrajectoryAsync(carouselTrajectory);
                    break;
                case GO_TO_CAROUSEL:
                    if (!drive.isBusy()) {
                        state = MachineState.SPIN_DUCK;
                        spinTimer.reset();
                    }
                    break;
                case SPIN_DUCK:
                    //Spin the duck then switch state to checking the barcode
                    useWheel(1);
                    if (spinTimer.time() >= spinTime) {
                        state = MachineState.GO_TO_BARCODE;
                        useWheel(0);
                        drive.followTrajectoryAsync(barcodeTrajectory);
                    }
                    break;
                case GO_TO_BARCODE:
                    if (!drive.isBusy()) {
                        state = MachineState.CHECK_BARCODE_1;
                    }
                    break;
                case CHECK_BARCODE_1:
                    boolean barcode1 = checkBarcode(20);
                    if (barcode1) {
                        state = MachineState.GO_TO_SHIPPING_HUB;
                        level = ElevatorLevel.LEVEL_ONE;
                        checkedBarcode = true;
                        //drive.followTrajectoryAsync();
                    } else {
                        state = MachineState.CHECK_BARCODE_2;
                        drive.followTrajectoryAsync(barcode2Trajectory);
                    }

                    break;
                case CHECK_BARCODE_2:
                    if (!drive.isBusy()) {
                        //check barcode
                        boolean barcode2 = checkBarcode(20);
                        if (barcode2) {
                            level = ElevatorLevel.LEVEL_TWO;
                            checkedBarcode = true;
                        }
                        state = MachineState.GO_TO_SHIPPING_HUB;
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    if (checkedBarcode) {
                        checkedBarcode = false;
                    } else level = ElevatorLevel.LEVEL_THREE;
                    state = MachineState.LIFT_FREIGHT;
                    break;
                case LIFT_FREIGHT:
                    useElevatorAsync(level);
                    state = MachineState.DUMP_FREIGHT;
                    break;
                case DUMP_FREIGHT:
                    if (!isElevatorBusy()){
                        useClaws(false);
                        state = MachineState.GO_TO_STORAGE_UNIT;
                        //drive.followTrajectoryAsync(...)
                        useElevatorAsync(ElevatorLevel.START);
                    }
                    break;
                case GO_TO_STORAGE_UNIT:
                    //Drive the robot to the required location in the storage unit
                    if (runtime.time() > 26) state = MachineState.IDLE;
                    else state = MachineState.IDENTIFY_FREIGHT;
                    break;
                case IDENTIFY_FREIGHT:
                    //Use the 2m sensor to identify the freight and give the required pose to pick up the freight
                    state = MachineState.POSITION_TO_PICK_FREIGHT;
                    break;
                case POSITION_TO_PICK_FREIGHT:
                    //Drive the robot to the required pose
                    Vector2d freightLocation = new Vector2d(0, 0);
                    Trajectory goToFreight = drive.trajectoryBuilder(drive.getPoseEstimate()).lineToConstantHeading(freightLocation).build();
                    state = MachineState.PICK_FREIGHT;
                    break;
                case PICK_FREIGHT:
                    //Actually pick up freight
                    if (!drive.isBusy()) {
                        state = MachineState.GO_TO_SHIPPING_HUB;
                    }
                    break;
                case IDLE:
                    if (!drive.isBusy()) idle();
                    break;
                default:
                    //This should never happen but in case it does we set it to idle
                    state = MachineState.IDLE;
                    break;
            }
        }
    }
}
