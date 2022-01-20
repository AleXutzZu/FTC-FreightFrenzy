package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class RobotLimbControls {
    protected RobotHardware robotHardware = RobotHardware.getInstance();

    /**
     * Opens the claws of the arm (at full speed)
     */
    public abstract void useClaws();

    /**
     * Rotates the carousel wheel (at full speed)
     */
    public abstract void rotateWheel();

    /**
     * Moves the arm up or down
     * @param servoPosition float value between -1.0 and 1.0 that dictates how fast the arm should retract
     */
    public abstract void useArm(float servoPosition);

    /**
     * Brings the elevator up or puts it down
     * @param motorPower float value between -1.0 and 1.0 that dictates how fast the elevator should come up
     */
    public abstract void useElevator(float motorPower);
}
