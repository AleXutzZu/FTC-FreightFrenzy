package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class RobotLimbControls {
    protected RobotHardware robotHardware = new RobotHardware();

    protected RobotLimbControls(HardwareMap hardwareMap){
        robotHardware.init(hardwareMap);
    }

    //TODO Add methods responsible for controlling the claw and elevators
}
