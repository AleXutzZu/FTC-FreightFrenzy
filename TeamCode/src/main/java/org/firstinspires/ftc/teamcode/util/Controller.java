package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class Controller {
    protected RobotHardware robotHardware = new RobotHardware();

    protected Controller() {
        robotHardware.init(hardwareMap);
    }

    /**
     * Drives the robot forwards by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveForward(float motorPower);

    /**
     * Drives the robot backwards by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */

    public abstract void driveBackward(float motorPower);

    /**
     * Drives the robot left by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveLeft(float motorPower);

    /**
     * Drives the robot right by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveRight(float motorPower);

    /**
     * Drives the robot on the right diagonal going forwards by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveDiagonallyRightForward(float motorPower);

    /**
     * Drives the robot on the left diagonal going forwards by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveDiagonallyLeftForward(float motorPower);

    /**
     * Drives the robot on the right diagonal going backwards by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveDiagonallyRightBackward(float motorPower);

    /**
     * Drives the robot on the left diagonal going backwards by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void driveDiagonallyLeftBackward(float motorPower);

    /**
     * Steers the robot to the left by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void steerLeft(float motorPower);

    /**
     * Steers the robot to the right by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void steerRight(float motorPower);

    /**
     * Stops the motors like a handbrake.
     */
    public abstract void stopMotors();
}
