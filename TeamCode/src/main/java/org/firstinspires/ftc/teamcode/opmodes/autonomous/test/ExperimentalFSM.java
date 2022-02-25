package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Experimental Finite State Machine", group = "Testing Purposes")
public class ExperimentalFSM extends AutonomousControl {
    private enum MachineState {
        START,
        GO_TO_CAROUSEL,
        SPIN_DUCK,
        GO_TO_BARCODE, CHECK_BARCODE_1, CHECK_BARCODE_2,
        GO_TO_SHIPPING_HUB, LIFT_FREIGHT, DUMP_FREIGHT,
        GO_TO_STORAGE_UNIT, IDENTIFY_FREIGHT, POSITION_TO_PICK_FREIGHT, PICK_FREIGHT,
        IDLE
    }

    @Override
    protected void run() {
        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));
        ElapsedTime runtime = new ElapsedTime();
        MachineState state = MachineState.START;
        int level = 3;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory carouselTrajectory = drive.trajectoryBuilder(startPose).lineToLinearHeading(new Pose2d(-55, -49, Math.toRadians(45))).build();
        Trajectory barcodeTrajectory = drive.trajectoryBuilder(carouselTrajectory.end()).lineToLinearHeading(new Pose2d(-45, -48, Math.toRadians(0))).build();
        Trajectory barcode2Trajectory = drive.trajectoryBuilder(barcodeTrajectory.end()).lineTo(new Vector2d(-35, -48)).build();

        boolean checkedBarcode = false;
        while (opModeIsActive()) {
            switch (state) {
                case START:
                    drive.setPoseEstimate(startPose);
                    state = MachineState.GO_TO_CAROUSEL;
                    break;
                case GO_TO_CAROUSEL:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(carouselTrajectory);
                        state = MachineState.SPIN_DUCK;
                    }
                    // drive the robot to the carousel and check if the robot has done moving
                    break;
                case SPIN_DUCK:
                    //Spin the duck then switch state to checking the barcode
                    if (!drive.isBusy()) {
                        //spin the motor
                        state = MachineState.GO_TO_BARCODE;
                    }
                    break;
                case GO_TO_BARCODE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(barcodeTrajectory);
                        state = MachineState.CHECK_BARCODE_1;
                    }
                    break;
                case CHECK_BARCODE_1:
                    if (!drive.isBusy()) {
                        boolean barcode1 = true;//todo
                        if (barcode1) {
                            state = MachineState.GO_TO_SHIPPING_HUB;
                            level = 1;
                            checkedBarcode = true;
                            //drive.followTrajectoryAsync();
                        } else {
                            state = MachineState.CHECK_BARCODE_2;
                            drive.followTrajectoryAsync(barcode2Trajectory);
                        }
                    }
                    break;
                case CHECK_BARCODE_2:
                    if (!drive.isBusy()) {
                        //check barcode
                        boolean barcode2 = false;//todo
                        if (barcode2) {
                            level = 2;
                            checkedBarcode = true;
                        }
                        state = MachineState.GO_TO_SHIPPING_HUB;
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    if (checkedBarcode) {
                        checkedBarcode = false;
                    } else level = 3;
                    state = MachineState.LIFT_FREIGHT;
                    break;
                case LIFT_FREIGHT:
                    state = MachineState.DUMP_FREIGHT;
                    break;
                case DUMP_FREIGHT:
                    //Go with the elevator to the required level
                    //Open claws
                    state = MachineState.GO_TO_STORAGE_UNIT;
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
