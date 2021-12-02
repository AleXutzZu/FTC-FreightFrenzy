package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class RobotLimbControls {
    protected RobotHardware robotHardware = RobotHardware.getInstance();

    /**
     * Opens the claws of the arm (at full speed)
     */
    public abstract void openClaws();

    /**
     * Closes the claws of the arm (at full speed)
     */
    public abstract void closeClaws();

    /**
     * Rotates the carousel wheel (at full speed)
     */
    public abstract void rotateWheel();

    /**
     * Brings the arm up with the specified power
     * @param servoPosition float value between 0.0 and 1.0 that dictates how fast the arm should retract
     */
    public abstract void bringArmUp(float servoPosition);

    /**
     * Puts the arm down with the specified power
     * @param servoPosition float value between 0.0 and 1.0 that dictates how fast the arm should go down
     */
    public abstract void putArmDown(float servoPosition);

    /**
     * Brings the elevator up with the specified power
     * @param motorPower float value between 0.0 and 1.0 that dictates how fast the elevator should come up
     */
    public abstract void elevatorUp(float motorPower);

    /**
     * Brings the elevator down with the specified power
     * @param motorPower float value between 0.0 and 1.0 that dictates how fast the elevator should come down
     */
    public abstract void elevatorDown(float motorPower);
}
