package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an OpMode.
 *
 * This class can be used to define all the specific hardware for the Perpetuum Mobile Robot.
 *
 * @Motors_for_the_mecanum_movement_of_the_robot
 * Motor channel:  Right front motor:                                       "right_front"
 * Motor channel:  Right back motor:                                        "right_back"
 * Motor channel:  Left front motor:                                        "left_front"
 * Motor channel:  Left back motor:                                         "right_back"
 *
 */


public class RobotHardware {
    /*
    Public OpMode members
     */
    public DcMotor rightFrontMotor = null;                   //right_front
    public DcMotor rightBackMotor = null;                    //right_back
    public DcMotor leftFrontMotor = null;                    //left_front
    public DcMotor leftBackMotor = null;                     //left_back

    public RobotHardware() {

    }

    public void init(HardwareMap hardwareMap) {

        /*
         Defining motors used for controlling the movement
         */
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor.setPower(0f);
        rightBackMotor.setPower(0f);
        leftBackMotor.setPower(0f);
        leftFrontMotor.setPower(0f);
    }

    /**
     * Sets the corresponding run mode across all four motors
     * @param runMode new run mode
     * @see DcMotor.RunMode#RUN_TO_POSITION
     * @see DcMotor.RunMode#STOP_AND_RESET_ENCODER
     * @see DcMotor.RunMode#RUN_USING_ENCODER
     */
    public void setMotorModes(DcMotor.RunMode runMode){
        rightFrontMotor.setMode(runMode);
        rightBackMotor.setMode(runMode);
        leftFrontMotor.setMode(runMode);
        leftBackMotor.setMode(runMode);
    }
}
