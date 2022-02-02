package org.firstinspires.ftc.teamcode.util;


import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class RobotLimbControls {
    /**
     * Servo position to bring the arms up (Arm base Servo)
     */
    protected static final float ARM_UP = 0.8f;
    /**
     * Servo position to put the arm down (Arm base Servo)
     */
    protected static final float ARM_DOWN = 0.3f;
    /**
     * Servo position to close the claws (Claw servos)
     */
    protected static final float CLAWS_CLOSED = 1f;
    /**
     * Servo position to open the claws (Claw servos)
     */
    protected static final float CLAWS_OPENED = 0f;

    /**
     * Elevator tick at which the motor should stop to prevent the string from
     * going in the opposite direction, resulting in inverting the elevator controls.
     */
    protected static final int ELEVATOR_THRESHOLD = 800;
    /**
     * Robot hardware instance to access the servos & motors
     */
    protected RobotHardware robotHardware = RobotHardware.getInstance();

    /**
     * Opens the claws of the arm (at full speed)
     */
    public abstract void useClaws(boolean stance);

    /**
     * Rotates the carousel wheel (at full speed)
     */
    public abstract void rotateWheel();

    /**
     * Moves the arm up or down
     *
     * @param stance Boolean value representing arm stance (true = up, false = down)
     */
    public abstract void useArm(boolean stance);

    /**
     * Brings the elevator up or puts it down
     *
     * @param motorPower float value between -1.0 and 1.0 that dictates how fast the elevator should come up
     */
    public abstract void useElevator(float motorPower);

    /**
     * Returns the hardware of the robot
     *
     * @return never-null hardware class with all installed devices
     */
    public @NonNull RobotHardware getRobotHardware() {
        return robotHardware;
    }
}
