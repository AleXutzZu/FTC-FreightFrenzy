package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.teamcode.control.Limbs;
import org.firstinspires.ftc.teamcode.control.Movements;

public abstract class Gamepads {
    protected final Movements robotMovements = Movements.getInstance();
    protected final Limbs robotLimbs = Limbs.getInstance();
    /**
     * This method will drive the robot on the field
     * @return the orientation of the robot
     */
    public abstract Direction drive();

    /**
     * This method will use the claw and elevator
     */
    public abstract void useLimbs();

    /**
     * Getter for robot movements in case functionality is inaccessible through the gamepad.
     * @return robotMovements object with all methods of driving
     */
    public Movements getRobotMovements() {
        return robotMovements;
    }
}
