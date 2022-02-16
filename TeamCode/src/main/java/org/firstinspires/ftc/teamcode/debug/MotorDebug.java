package org.firstinspires.ftc.teamcode.debug;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Debug;

public class MotorDebug implements Debug {
    private final RobotHardware robotHardware = RobotHardware.getInstance();

    @Override
    public void update(@NonNull Telemetry telemetry) {

    }
}
