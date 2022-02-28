package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * <h1>This class can be used to define all the specific hardware for the Perpetuum Mobile Robot.</h1>
 * This is NOT an OpMode.
 * <br>
 * <p>Used Motors (driving & lift): <a href="https://www.pitsco.com/TETRIX-MAX-TorqueNADO-Motor-with-Encoder">TETRIX-MAX TorqueNADO Motor with Encoder</a></p>
 * <p>Used Motors (wheel): <a href="https://www.andymark.com/products/neverest-classic-40-gearmotor">NeveRest Classic 40 Gearmotor</a></p>
 * <h2>Motors</h2>
 * <h3>Motors for driving the robot</h3>
 * <pre>Right front motor:                                  <i>"right_front"</i></pre>
 * <pre>Right back motor:                                   <i>"right_back"</i></pre>
 * <pre>Left front motor:                                   <i>"left_front"</i></pre>
 * <pre>Left back motor:                                    <i>"right_back"</i></pre>
 * <h3>Motor for using the elevator</h3>
 * <pre>Elevator motor:                                 <i>"elevator_motor"</i></pre>
 * <h3>Motor to spin the carousel</h3>
 * <pre>Wheel spinning motor:                               <i>"wheel_motor"</i></pre>
 * <br>
 * <h2>Servos</h2>
 * <h3>Arm Servos</h3>
 * <pre>Arm base                                            <i>"arm_base"</i></pre>
 * <h3>Claw Servos</h3>
 * <pre>Left claw                                           <i>"left_claw"</i></pre>
 * <pre>Right claw                                          <i>"right_claw"</i></pre>
 * <br>
 * <h2>Sensors and misc</h2>
 * <h3>2M Distance Sensors</h3>
 * <pre>Left side sensor                                    <i>"left_2m"</i></pre>
 * <pre>Right side sensor                                   <i>"right_2m</i></pre>
 * <h3>Misc</h3>
 * <pre>BNO55IMU Gyroscope                                  <i>"imu"</i></pre>
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
    Inertial Measurement Unit
     */
    private BNO055IMU gyroscope = null;                      //imu

    /*
    Servos
     */
    private Servo armBase = null;                           //arm_base
    private Servo leftClaw = null;                          //left_claw
    private Servo rightClaw = null;                         //right_claw

    /*
    Sensors
     */
    private Rev2mDistanceSensor leftDistanceSensor = null; //left_2m
    private Rev2mDistanceSensor rightDistanceSensor = null; //right_2m

    private static RobotHardware instance = null;

    /**
     * Gets the instance of the hardware class
     *
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
     * Types of OpModes for initialization
     */
    private enum OpModeType {
        AUTONOMOUS, TELE_OP
    }

    /**
     * Initializes all hardware necessary for TeleOps.
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initTeleOp(@NonNull HardwareMap hardwareMap) {
        initMotorsTeleOp(hardwareMap);
        initServos(hardwareMap);
    }

    /**
     * Initializes all hardware necessary for Autonomous OpModes
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initAutonomous(@NonNull HardwareMap hardwareMap) {
        initMotorsAutonomous(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);
        initGyro(hardwareMap);
    }

    /**
     * Initializes the drivetrain motors with their specified run-modes
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     * @param opModeType  the type of OpMode for which the initialization is done
     * @throws IllegalStateException if the opModeType parameter is invalid
     */
    private void initDriveTrainMotors(@NonNull HardwareMap hardwareMap, @NonNull OpModeType opModeType) throws IllegalStateException {
         /*
         Defining motors used for movement
         */
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        switch (opModeType) {
            case AUTONOMOUS:
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case TELE_OP:
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + opModeType);
        }

        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
        Doing initialization of power
         */
        rightFrontMotor.setPower(0f);
        rightBackMotor.setPower(0f);
        leftBackMotor.setPower(0f);
        leftFrontMotor.setPower(0f);
    }

    /**
     * Initializes motors installed on the robot for the TeleOp mode(it sets the drivetrain motors to run without encoders)
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initMotorsTeleOp(@NonNull HardwareMap hardwareMap) {
        initDriveTrainMotors(hardwareMap, OpModeType.TELE_OP);
        initElevator(hardwareMap);
        initWheelMotor(hardwareMap);
    }

    /**
     * Initializes motors installed on the robot for the autonomous mode(it sets the drivetrain motors to run using encoders)
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initMotorsAutonomous(@NonNull HardwareMap hardwareMap) {
        initDriveTrainMotors(hardwareMap, OpModeType.AUTONOMOUS);
        initElevator(hardwareMap);
        initWheelMotor(hardwareMap);
    }

    /**
     * Initializes the elevator on the drivetrain to run with encoders regardless of OpMode (TeleOp or Autonomous)
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initElevator(@NonNull HardwareMap hardwareMap) {
         /*
        Defining the motor for the elevator
         */
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setPower(0f);
    }

    /**
     * Initializes the wheel motor on the drivetrain to run without encoders regardless of OpMode (TeleOp or Autonomous)
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initWheelMotor(@NonNull HardwareMap hardwareMap) {
        /*
        Defining the motor to rotate the carousel
        There is no need for an encoder
         */
        wheelMotor = hardwareMap.get(DcMotor.class, "wheel_motor");
        wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelMotor.setPower(0f);
    }

    /**
     * Initializes servos installed on the robot
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initServos(@NonNull HardwareMap hardwareMap) {
        /*
        Servos
         */
        armBase = hardwareMap.get(Servo.class, "arm_base");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        armBase.setDirection(Servo.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Initializes sensors on the robot for the autonomous period
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     */
    public void initSensors(@NonNull HardwareMap hardwareMap) {
        /*
        Defining the sensors used
         */
        leftDistanceSensor = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "left_2m");
        rightDistanceSensor = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "right_2m");
    }

    /**
     * Initializes gyroscope for the autonomous period
     *
     * @param hardwareMap never null map with the configuration from the robot controller app
     * @throws RuntimeException if the gyro fails to initialize
     */
    public void initGyro(@NonNull HardwareMap hardwareMap) throws RuntimeException {
        /*
        Gyroscope setup
         */
        gyroscope = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        if (!gyroscope.initialize(imuParameters)) {
            throw new RuntimeException("Could not initialize Gyroscope");
        }
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

    public BNO055IMU getGyroscope() {
        return gyroscope;
    }

    public Rev2mDistanceSensor getLeftDistanceSensor() {
        return leftDistanceSensor;
    }

    public Rev2mDistanceSensor getRightDistanceSensor() {
        return rightDistanceSensor;
    }
}
