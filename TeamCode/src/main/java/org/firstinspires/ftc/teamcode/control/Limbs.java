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
    public void openClaws() {

    }

    @Override
    public void closeClaws() {

    }

    @Override
    public void rotateWheel() {
        robotHardware.getWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getWheelMotor().setPower(1f);
    }

    @Override
    public void bringArmUp(float servoPosition) {
        robotHardware.getArmBase().setDirection(Servo.Direction.FORWARD);
        robotHardware.getArmBase().setPosition(servoPosition);
    }

    @Override
    public void putArmDown(float servoPosition) {
        robotHardware.getArmBase().setDirection(Servo.Direction.REVERSE);
        robotHardware.getArmBase().setPosition(servoPosition);
    }

    @Override
    public void elevatorUp(float motorPower) {
        robotHardware.getElevatorMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.getElevatorMotor().setPower(motorPower);
    }

    @Override
    public void elevatorDown(float motorPower) {
        robotHardware.getElevatorMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getElevatorMotor().setPower(motorPower);
    }
}
