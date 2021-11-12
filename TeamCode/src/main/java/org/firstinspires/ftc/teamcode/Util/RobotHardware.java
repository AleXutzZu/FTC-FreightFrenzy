package org.firstinspires.ftc.teamcode.Util;

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
    public DcMotor rightFront = null;                   //right_front
    public DcMotor rightBack = null;                    //right_back
    public DcMotor leftFront = null;                    //left_front
    public DcMotor leftBack = null;                     //left_back

    public RobotHardware() {

    }

    public void init(HardwareMap hardwareMap) {

        /*
         Defining motors used for controlling the movement
         */
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
    }
}
