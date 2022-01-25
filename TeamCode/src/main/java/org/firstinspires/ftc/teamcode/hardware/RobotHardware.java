package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * <h1>This class can be used to define all the specific hardware for the Perpetuum Mobile Robot.</h1>
 * This is NOT an OpMode.
 * <br>
 * <p>Used Motors (driving): <a href="https://www.pitsco.com/TETRIX-MAX-TorqueNADO-Motor-with-Encoder">TETRIX-MAX TorqueNADO Motor with Encoder</a></p>
 * <p>Used Motors (lift and wheel): <a href="https://www.andymark.com/products/neverest-classic-40-gearmotor">NeveRest Classic 40 Gearmotor</a></p>
 * <h2>Motors for driving the robot</h2>
 * <pre>Right front motor:                                  <i>"right_front"</i></pre>
 * <pre>Right back motor:                                   <i>"right_back"</i></pre>
 * <pre>Left front motor:                                   <i>"left_front"</i></pre>
 * <pre>Left back motor:                                    <i>"right_back"</i></pre>
 * <br>
 * <h2>Motor for using the elevator</h2>
 * <pre>Elevator motor:                                 <i>"elevator_motor"</i></pre>
 * <br>
 * <h2>Motor to spin the carousel</h2>
 * <pre>Wheel spinning motor:                               <i>"wheel_motor"</i></pre>
 * <br>
 * <h2>Servos</h2>
 * <pre>Arm base                                            <i>"arm_base"</i></pre>
 * <pre>Left claw                                           <i>"left_claw"</i></pre>
 * <pre>Right claw                                          <i>"right_claw"</i></pre>
 * <br>
 * <h2>Sensors and misc</h2>
 */


public class RobotHardware {
    /*
    Motors
     */
    private DcMotor rightFrontMotor = null;                  //right_front
    private DcMotor rightBackMotor = null;                   //right_back
    private DcMotor leftFrontMotor = null;                   //left_front
    private DcMotor leftBackMotor = null;                    //left_back
    private DcMotor elevatorMotor = null;                    //elevator_motor
    private DcMotor wheelMotor = null;                       //wheel_motor

    /*
    Servos
     */
    private Servo armBase = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;

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

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        Defining the motor for the elevator
         */
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        Defining the motor to rotate the carousel
        There is no need for an encoder
         */
        wheelMotor = hardwareMap.get(DcMotor.class, "wheel_motor");

        wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        Servos
         */
        armBase = hardwareMap.get(Servo.class, "arm_base");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        /*
        Setting the power to 0f for starters
         */
        rightFrontMotor.setPower(0f);
        rightBackMotor.setPower(0f);
        leftBackMotor.setPower(0f);
        leftFrontMotor.setPower(0f);
        elevatorMotor.setPower(0f);
        wheelMotor.setPower(0f);
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

    public Servo getArmBase() {
        return armBase;
    }

    public Servo getLeftClaw() {
        return leftClaw;
    }

    public Servo getRightClaw() {
        return rightClaw;
    }
}
