package org.firstinspires.ftc.teamcode.control;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.RobotLimbControls;

public class Limbs extends RobotLimbControls {
    private static Limbs instance = null;
    private boolean claws = false;
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
    public void useClaws(boolean stance) {
        robotHardware.getLeftClaw().setDirection(Servo.Direction.FORWARD);
        robotHardware.getRightClaw().setDirection(Servo.Direction.FORWARD);

        if (stance) {
            robotHardware.getLeftClaw().setPosition(CLAWS_OPENED);
            robotHardware.getRightClaw().setPosition(CLAWS_OPENED);
        } else {
            robotHardware.getLeftClaw().setPosition(CLAWS_CLOSED);
            robotHardware.getRightClaw().setPosition(CLAWS_CLOSED);
        }
    }

    @Override
    public void rotateWheel() {
        robotHardware.getWheelMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        robotHardware.getWheelMotor().setPower(1f);
        robotHardware.getWheelMotor().setPower(0f);
    }

    @Override
    public void useArm(boolean stance) {
        robotHardware.getArmBase().setDirection(Servo.Direction.FORWARD);
        robotHardware.getArmBase().setPosition(stance ? ARM_UP : ARM_DOWN);
    }

    @Override
    public void useElevator(float motorPower) {
        robotHardware.getElevatorMotor().setDirection(motorPower >= 0f ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        robotHardware.getElevatorMotor().setPower(Math.abs(motorPower));
    }

    public boolean isClaws() {
        return claws;
    }

    public void setClaws(boolean claws) {
        this.claws = claws;
    }
}
