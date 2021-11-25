package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an OpMode.
 *
 * This class can be used to define all the specific hardware for the Perpetuum Mobile Robot.
 *
 * Motors for driving the robot
 * Motor channel:  Right front motor:                                       "right_front"
 * Motor channel:  Right back motor:                                        "right_back"
 * Motor channel:  Left front motor:                                        "left_front"
 * Motor channel:  Left back motor:                                         "right_back"
 *
 * <p>Motor for using the elevator</p>
 * <p>Motor channel:  Elevator motor:                                          "elevator_motor"</p>
 *
 * <p>Motor to spin the carousel</p>
 * <p>Motor channel:  Wheel spinning motor                                     "wheel_motor"</p>
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
    private DcMotor elevatorMotor = null;                    //elevator_motor
    private DcMotor wheelMotor = null;                       //wheel_motor

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
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        wheelMotor = hardwareMap.get(DcMotor.class, "wheel_motor");

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setPower(0f);
        rightBackMotor.setPower(0f);
        leftBackMotor.setPower(0f);
        leftFrontMotor.setPower(0f);
    }

    public DcMotor getElevatorMotor() {
        return elevatorMotor;
    }

    public DcMotor getWheelMotor() {
        return wheelMotor;
    }
}
