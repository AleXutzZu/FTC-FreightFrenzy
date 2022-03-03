package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;

public abstract class AutonomousUtil extends OpMode {
    protected final RobotHardware robotHardware = RobotHardware.getInstance();

    protected SampleMecanumDrive drive;

    @Override
    public void init() {
        robotHardware.initAutonomous(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    /**
     * Powers the carousel wheel making it spin
     *
     * @param power given power. If it is not within [-1.0, 1.0] it will be capped to the closest value from that interval.
     */
    protected void useWheel(double power) {
        power = Range.clip(power, -1d, 1d);
        robotHardware.getWheelMotor().setPower(power);
    }

    /**
     * Checks the distance sensor to see if anything is in range. Should be used when the robot
     * is positioned in parallel to the barcode positions, with the sensor facing towards the barcode
     *
     * @param distance how close should the object be
     * @return whether or not there is an object within distance in centimetres
     */
    protected boolean checkBarcode(double distance) {
        return robotHardware.getLeftDistanceSensor().getDistance(DistanceUnit.CM) < distance;
    }

    /**
     * Brings the elevator to the specified level asynchronously
     *
     * @param level specified level
     */
    protected void useElevatorAsync(@NonNull Constants.ElevatorLevels level) {
        int targetPos = (int) (level.getDistance() * Constants.ELEVATOR_TICKS_PER_CENTIMETRE);
        robotHardware.getElevatorMotor().setTargetPosition(targetPos);
        robotHardware.getElevatorMotor().setPower(1);
        robotHardware.getElevatorMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void useClaws(boolean closeClaws) {
        if (closeClaws) {
            robotHardware.getRightClaw().setPosition(Constants.CLAWS_CLOSED);
            robotHardware.getLeftClaw().setPosition(Constants.CLAWS_CLOSED);
        } else {
            robotHardware.getLeftClaw().setPosition(Constants.CLAWS_OPENED);
            robotHardware.getRightClaw().setPosition(Constants.CLAWS_OPENED);
        }
    }

    protected void useArm(boolean armUp) {
        if (armUp) robotHardware.getArmBase().setPosition(Constants.ARM_UP);
        else robotHardware.getArmBase().setPosition(Constants.ARM_DOWN);
    }

    /**
     * Check if the elevator is busy
     *
     * @return whether the elevator has done retreating or advancing to the required position
     */
    protected boolean isElevatorBusy() {
        return robotHardware.getElevatorMotor().isBusy();
    }

    /**
     * Check if the elevator is at the start
     *
     * @return true if it is, false otherwise
     */
    protected boolean isElevatorAtStart() {
        return robotHardware.getElevatorMotor().getCurrentPosition() == 0;
    }
}
