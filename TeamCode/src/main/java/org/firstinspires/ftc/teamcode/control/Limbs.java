package org.firstinspires.ftc.teamcode.control;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.RobotLimbControls;

public class Limbs extends RobotLimbControls {
    private static Limbs instance = null;

    //prevent instantiation
    private Limbs() {

    }

    /**
     * Get the Limbs instance
     *
     * @return the instance of the class
     */
    public static Limbs getInstance() {
        if (instance == null) instance = new Limbs();
        return instance;
    }

    @Override
    public void useClaws() {

    }

    @Override
    public void rotateWheel() {
        robotHardware.getWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getWheelMotor().setPower(1f);
        robotHardware.getWheelMotor().setPower(0f);
    }

    @Override
    public void useArm(float servoPosition) {
        robotHardware.getArmBase().setDirection(servoPosition >= 0f ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        robotHardware.getArmBase().setPosition(servoPosition);
    }

    @Override
    public void useElevator(float motorPower) {
        robotHardware.getElevatorMotor().setDirection(motorPower >= 0f ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        robotHardware.getElevatorMotor().setPower(Math.abs(motorPower));
    }
}
