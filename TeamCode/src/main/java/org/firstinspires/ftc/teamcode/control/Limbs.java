package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

    /**
     * Initializes the Limbs object with the hardware map
     *
     * @param hardwareMap never null map responsible for configuring the robot
     */
    public void init(@NonNull HardwareMap hardwareMap) {
        robotHardware.init(hardwareMap);
    }

    @Override
    public void openClaws() {

    }

    @Override
    public void closeClaws() {

    }

    @Override
    public void rotateWheel() {

    }

    @Override
    public void bringArmUp(float motorPower) {

    }

    @Override
    public void putArmDown(float motorPower) {

    }

    @Override
    public void elevatorUp(float motorPower) {

    }

    @Override
    public void elevatorDown(float motorPower) {

    }
}
