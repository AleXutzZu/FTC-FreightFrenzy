package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@FunctionalInterface
public interface Debug {
    /**
     * Updates the debug console with telemetry
     *
     * @param telemetry the telemetry console to print to, never null
     */
    void update(@NonNull Telemetry telemetry);
}
