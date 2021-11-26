package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an OpMode.
 * <p>
 * This class can be used to define all the specific hardware for the Perpetuum Mobile Robot.
 *
 * Motors for driving the robot
 * <p>Motor channel:  Right front motor:                                       "right_front"</p>
 * <p>Motor channel:  Right back motor:                                        "right_back"</p>
 * <p>Motor channel:  Left front motor:                                        "left_front"</p>
 * <p>Motor channel:  Left back motor:                                         "right_back"</p>
 *
 * <p>Motor for using the elevator</p>
 * <p>Motor channel:  Elevator motor:                                          "elevator_motor"</p>
 *
 * <p>Motor to spin the carousel</p>
 * <p>Motor channel:  Wheel spinning motor:                                    "wheel_motor"</p>
 *
 */


public class RobotHardware {
    /*
    Motors
     */
    private DcMotor rightFrontMotor = null;                   //right_front
    private DcMotor rightBackMotor = null;                    //right_back
    private DcMotor leftFrontMotor = null;                    //left_front
    private DcMotor leftBackMotor = null;                     //left_back
    private DcMotor elevatorMotor = null;                    //elevator_motor
    private DcMotor wheelMotor = null;                       //wheel_motor

    private static RobotHardware instance = null;

    /**
     * Gets the instance of the hardware class
     * @return the current instance
     */
    public static RobotHardware getInstance() {
        if (instance == null) instance = new RobotHardware();
        return instance;
    }
    //prevent direct instantiation
    private RobotHardware() {

    }

    /**
     * Initializes motors, servos and other hardware installed on the robot
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void init(@NonNull HardwareMap hardwareMap) {
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

    public DcMotor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotor getRightBackMotor() {
        return rightBackMotor;
    }

    public DcMotor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotor getLeftBackMotor() {
        return leftBackMotor;
    }

    public DcMotor getElevatorMotor() {
        return elevatorMotor;
    }

    public DcMotor getWheelMotor() {
        return wheelMotor;
    }
}
