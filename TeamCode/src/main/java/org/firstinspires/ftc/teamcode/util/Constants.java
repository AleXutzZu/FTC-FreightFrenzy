package org.firstinspires.ftc.teamcode.util;

/**
 * Various constants for various tasks including servo positions, tick/cm ratios and different driving powers
 */
public final class Constants {
    /*
    TODO
        - Possibly update values to match goBILDA motors and wheels.
        - Currently using TETRIX Motors and HD Mecanum Wheels
     */
    /**
     * Servo position to bring the arms up (Arm base Servo)
     */
    public static final double ARM_UP = 1f;

    /**
     * Servo position to put the arm down (Arm base Servo)
     */
    public static final double ARM_DOWN = 0.5f;

    /**
     * Servo position to close the claws (Claw servos)
     */
    public static final double CLAWS_CLOSED = 1f;

    /**
     * Servo position to open the claws (Claw servos)
     */
    public static final double CLAWS_OPENED = 0f;

    /**
     * Dictates how small the output from the joystick/trigger should be
     */
    public static final double POWER_RATIO = 2;

    /**
     * Power input for rotating around the central axis
     */
    public static final double ROTATION_POWER = 1;

    /**
     * Power input for sliding operation (left or right)
     */
    public static final double SLIDING_POWER = 1;

    /**
     * Power used when the robot is driving straight (during autonomous)
     */
    public static final double DRIVING_POWER = 1d;

    /**
     * Power input for diagonal driving (in all 4 directions)
     */
    public static final double DIAGONAL_DRIVING_POWER = 1;

    /**
     * Tetrix motor tick rate, according to their spec sheet
     * <p><a href="https://www.pitsco.com/TETRIX-MAX-TorqueNADO-Motor-with-Encoder">TETRIX-MAX TorqueNADO Motor with Encoder</a></p>
     */
    public static final int TETRIX_MOTOR_TICK_RATE = 1440;

    /**
     * Diameter for the inner circle to which the cord is tied (in cm)
     * <p><a href="https://www.servocity.com/1-25-winch-pulley/">1.25” Winch Pulley</a></p>
     */
    public static final double WINCH_PULLEY_DIAMETER = 3.175;

    /**
     * Ticks required by the elevator motor to advance the cord 1 cm
     */
    public static final double ELEVATOR_TICKS_PER_CENTIMETRE = TETRIX_MOTOR_TICK_RATE / (WINCH_PULLEY_DIAMETER * Math.PI);

    /**
     * Maximum ticks the elevator may go to before risking to break the cord. (About 30cm)
     */
    public static final int MAX_ELEVATOR_TICKS = (int) (70 * ELEVATOR_TICKS_PER_CENTIMETRE);

    /**
     * <p>Wheel diameter in <b>centimetres</b></p>
     * <a href= "https://www.andymark.com/products/4-in-hd-mecanum-wheel-set-options">HD Mecanum Wheels</a>
     */
    public static final double WHEEL_DIAMETER = 10.16;

    /**
     * <p>Ticks per centimetre based on motor and wheel specs</p>
     * <p>Defined as <b>MOTOR_TICK_RATE / WHEEL_DIAMETER</b></p>
     *
     * @see Constants#WHEEL_DIAMETER
     * @see Constants#TETRIX_MOTOR_TICK_RATE
     */
    public static final double DRIVING_TICKS_PER_CENTIMETRE = TETRIX_MOTOR_TICK_RATE / (WHEEL_DIAMETER * Math.PI);
}
