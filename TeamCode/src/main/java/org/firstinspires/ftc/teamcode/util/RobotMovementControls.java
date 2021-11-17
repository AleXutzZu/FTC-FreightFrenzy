package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class RobotMovementControls {
    protected RobotHardware robotHardware = new RobotHardware();

    protected RobotMovementControls(HardwareMap hardwareMap) {
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
     * Steers the robot to the forwards by the specified powers
     * If leftMotorsPower > rightMotorsPower the robot will steer to the left, otherwise to the right.
     *  @param leftMotorsPower float value between 0.0 and 1.0 representing the power given to the left motors. (Less power means less speed)
     * @param rightMotorsPower float value between 0.0 and 1.0 representing the power given to the right motors.
     */
    public abstract void steerForward(float leftMotorsPower, float rightMotorsPower);

    /**
     * Steers the robot backwards by the specified powers
     * If leftMotorsPower > rightMotorsPower the robot will steer to the right, otherwise to the left.
     *  @param leftMotorsPower float value between 0.0 and 1.0 representing the power given to the motors.
     * @param rightMotorsPower float value between 0.0 and 1.0 representing the power given to the motors. (Less power means less speed)
     */
    public abstract void steerBackward(float leftMotorsPower, float rightMotorsPower);

    /**
     * Rotates the robot on its center axis counter clockwise by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void rotateLeft(float motorPower);

    /**
     * Rotates the robot on its center axis clockwise by the specified power
     *
     * @param motorPower float value between 0.0 and 1.0 representing the power given to the motors (Less power means less speed)
     */
    public abstract void rotateRight(float motorPower);
    /**
     * Stops the motors like a handbrake.
     */
    public abstract void stopMotors();
}
