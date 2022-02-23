package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous(name = "Experimental Finite State Machine", group = "Testing Purposes")
public class ExperimentalFSM extends AutonomousControl {
    private enum MachineState {
        START, GO_TO_CAROUSEL, SPIN_DUCK, CHECK_BARCODE, GO_TO_SHIPPING_HUB, DUMP_FREIGHT, GO_TO_STORAGE_UNIT, IDENTIFY_FREIGHT, POSITION_TO_PICK_FREIGHT, PICK_FREIGHT, IDLE
    }

    @Override
    protected void run() {
        ElapsedTime runtime = new ElapsedTime();
        MachineState state = MachineState.START;
        int level = 3;
        boolean checkedBarcode = false;
        while (opModeIsActive()) {
            switch (state) {
                case START:
                    //Set pose estimate
                    state = MachineState.GO_TO_CAROUSEL;
                    break;
                case GO_TO_CAROUSEL:
                    // drive the robot to the carousel and check if the robot has done moving
                    state = MachineState.SPIN_DUCK;
                    break;
                case SPIN_DUCK:
                    //Spin the duck then switch state to checking the barcode
                    state = MachineState.CHECK_BARCODE;
                    break;
                case CHECK_BARCODE:
                    //Drive the robot to the 3 barcodes and check the barcodes then switch state
                    state = MachineState.GO_TO_SHIPPING_HUB;
                    level = 2;
                    checkedBarcode = true;
                    break;
                case GO_TO_SHIPPING_HUB:
                    if (checkedBarcode) {
                        checkedBarcode = false;
                    } else level = 3;
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
                    state = MachineState.PICK_FREIGHT;
                    break;
                case PICK_FREIGHT:
                    //Actually pick up freight
                    state = MachineState.GO_TO_SHIPPING_HUB;
                    break;
                case IDLE:
                    idle();
                    break;
                default:
                    //This should never happen but in case it does we set it to idle
                    state = MachineState.IDLE;
                    break;
            }
        }
    }
}
