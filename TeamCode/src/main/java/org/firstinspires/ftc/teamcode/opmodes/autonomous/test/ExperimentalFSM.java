package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.AutonomousUtil;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AbsolutePositions;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "Experimental Finite State Machine", group = "Testing Purposes")
public class ExperimentalFSM extends AutonomousUtil {
    private enum MachineState {
        GO_TO_CAROUSEL,
        SPIN_DUCK,
        CHECK_BARCODES,
        GO_TO_SHIPPING_HUB, LIFT_FREIGHT, BRING_ARM_DOWN, POSITION_TO_DUMP_FREIGHT, DUMP_FREIGHT, WAIT, RESET_POSITION,
        GO_TO_STORAGE_UNIT, IDENTIFY_FREIGHT, POSITION_TO_PICK_FREIGHT, PICK_FREIGHT,
        IDLE
    }

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime spinTimer = new ElapsedTime();
    private final ElapsedTime armTimer = new ElapsedTime();
    private final ElapsedTime clawTimer = new ElapsedTime();
    private static final double spinTime = 4.0d;
    private MachineState state;
    private final Pose2d startPose = AbsolutePositions.redStorageUnitStartPose;

    @Override
    public void init() {
        robotHardware.initAutonomous(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        /*
        Initializing trajectories
         */

        carouselTrajectory = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(AbsolutePositions.carousel, Math.toRadians(45)), Math.toRadians(-180)).build();

        barcodeTrajectory = drive.trajectoryBuilder(carouselTrajectory.end())
                .splineTo(AbsolutePositions.redStorageUnitBarcode1, Math.toRadians(0))
                .addSpatialMarker(AbsolutePositions.redStorageUnitBarcode1, () -> {
                    if (checkBarcode(20)) {
                        useElevatorAsync(Constants.ElevatorLevels.LEVEL_ONE);
                    }
                })
                .splineTo(AbsolutePositions.redStorageUnitBarcode2, 0)
                .addSpatialMarker(AbsolutePositions.redStorageUnitBarcode2, () -> {
                    if (checkBarcode(20)) {
                        useElevatorAsync(Constants.ElevatorLevels.LEVEL_TWO);
                    } else if (isElevatorAtStart())
                        useElevatorAsync(Constants.ElevatorLevels.LEVEL_THREE);
                })
                .build();

        shippingHubTrajectory = drive.trajectoryBuilder(barcodeTrajectory.end()).build();

        storageUnitTrajectory = drive.trajectoryBuilder(shippingHubTrajectory.end()).build();
    }

    @Override
    public void start() {
        state = MachineState.GO_TO_CAROUSEL;
        drive.followTrajectoryAsync(carouselTrajectory);
    }

    /*
    Trajectories
     */
    private Trajectory carouselTrajectory;
    private Trajectory barcodeTrajectory;
    private Trajectory shippingHubTrajectory;
    private Trajectory storageUnitTrajectory;

    @Override
    public void loop() {
        switch (state) {
            case GO_TO_CAROUSEL:
                if (!drive.isBusy()) {
                    state = MachineState.SPIN_DUCK;
                    spinTimer.reset();
                }
                break;
            case SPIN_DUCK:
                //Spin the duck then switch state to checking the barcode
                useWheel(-1);
                if (spinTimer.time() >= spinTime) {
                    state = MachineState.CHECK_BARCODES;
                    useWheel(0);
                    drive.followTrajectoryAsync(barcodeTrajectory);
                }
                break;
            case CHECK_BARCODES:
                if (!drive.isBusy()) {
                    state = MachineState.GO_TO_SHIPPING_HUB;
                    drive.followTrajectoryAsync(shippingHubTrajectory);
                }
                break;
            case GO_TO_SHIPPING_HUB:
                if (!drive.isBusy()) {
                    state = MachineState.LIFT_FREIGHT;
                }
                break;
            case LIFT_FREIGHT:
                if (isElevatorAtStart()) useElevatorAsync(Constants.ElevatorLevels.LEVEL_THREE);
                state = MachineState.BRING_ARM_DOWN;
                break;
            case BRING_ARM_DOWN:
                if (!isElevatorBusy()) {
                    useArm(false);
                    state = MachineState.POSITION_TO_DUMP_FREIGHT;
                    armTimer.reset();
                }
                break;
            case POSITION_TO_DUMP_FREIGHT:
                if (armTimer.milliseconds() > 200) {
                    Trajectory dumpFreight = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(5).build();
                    state = MachineState.DUMP_FREIGHT;
                    drive.followTrajectoryAsync(dumpFreight);
                }
                break;
            case DUMP_FREIGHT:
                if (!drive.isBusy()) {
                    useClaws(false);
                    clawTimer.reset();
                    state = MachineState.WAIT;
                }
                break;
            case WAIT:
                if (clawTimer.milliseconds() > 200) {
                    Trajectory resetTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(5).build();
                    drive.followTrajectoryAsync(resetTrajectory);
                    state = MachineState.RESET_POSITION;
                }
                break;
            case RESET_POSITION:
                if (!drive.isBusy()){
                    drive.followTrajectoryAsync(storageUnitTrajectory);
                    state = MachineState.GO_TO_STORAGE_UNIT;
                }
                break;
            case GO_TO_STORAGE_UNIT:
                if (!drive.isBusy()){
                    if (runtime.time() > 22) state = MachineState.IDLE;
                    else state = MachineState.IDENTIFY_FREIGHT;
                }
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
                return;
            default:
                //This should never happen but in case it does we set it to idle
                state = MachineState.IDLE;
                break;
        }

    }
}
