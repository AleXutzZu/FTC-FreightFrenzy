package org.firstinspires.ftc.teamcode.util;

/**
 * Various constants for various tasks including servo positions, tick/cm ratios and different driving powers
 */
public final class Constants {
    /**
     * Servo position to bring the arms up (Arm base Servo)
     */
    public static final double ARM_UP = 1f;

    /**
     * Servo position to put the arm down (Arm base Servo)
     */
    public static final double ARM_DOWN = 0.3f;

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
    public static final double ROTATION_POWER = 0.1;

    /**
     * Power input for sliding operation (left or right)
     */
    public static final double SLIDING_POWER = 1;

    /**
     * Power used when the robot is driving straight (during autonomous)
     */
    public static final double DRIVING_POWER = 0.05d;

    /**
     * Power input for diagonal driving (in all 4 directions)
     */
    public static final double DIAGONAL_DRIVING_POWER = 1;

    /**
     * goBILDA drivetrain motor tick rate, according to their spec sheet
     * <p><a href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/">5203 Series Yellow Jacket Planetary Gear Motor</a></p>
     */
    public static final double GOBILDA_DRIVETRAIN_MOTOR_TICK_RATE = 537.6;

    /**
     * goBILDA elevator motor tick rate, according to their spec sheet
     * <p><a href="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-435-rpm-3-3-5v-encoder/">5202 Series Yellow Jacket Planetary Gear Motor</a></p>
     */
    public static final double TETRIX_ELEVATOR_MOTOR_TICK_RATE = 1440;
    /**
     * Diameter for the inner circle to which the cord is tied (in cm)
     * <p><a href="https://www.servocity.com/1-25-winch-pulley/">1.25” Winch Pulley</a></p>
     */
    public static final double WINCH_PULLEY_DIAMETER = 3.175;

    /**
     * Ticks required by the elevator motor to advance the cord 1 cm
     */
    public static final double ELEVATOR_TICKS_PER_CENTIMETRE = TETRIX_ELEVATOR_MOTOR_TICK_RATE / (WINCH_PULLEY_DIAMETER * Math.PI);

    /**
     * Maximum ticks the elevator may go to before risking to break the cord. (About 30cm)
     */
    public static final int MAX_ELEVATOR_TICKS = (int) (55 * ELEVATOR_TICKS_PER_CENTIMETRE);

    /**
     * <p>Wheel diameter in <b>centimetres</b></p>
     * <a href= "https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/">96mm Mecanum Wheel Set</a>
     */
    public static final double WHEEL_DIAMETER = 9.6;

    /**
     * <p>Ticks per centimetre based on motor and wheel specs</p>
     * <p>Defined as <b>MOTOR_TICK_RATE / WHEEL_DIAMETER</b></p>
     *
     * @see Constants#WHEEL_DIAMETER
     * @see Constants#GOBILDA_DRIVETRAIN_MOTOR_TICK_RATE
     */
    public static final double DRIVING_TICKS_PER_CENTIMETRE = GOBILDA_DRIVETRAIN_MOTOR_TICK_RATE / (WHEEL_DIAMETER * Math.PI);
}
