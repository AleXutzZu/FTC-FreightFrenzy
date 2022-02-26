package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;

public abstract class AutonomousUtil extends LinearOpMode {
    protected final RobotHardware robotHardware = RobotHardware.getInstance();

    protected final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    /**
     * Elevator distances for each level in centimetres
     */
    protected enum ElevatorLevel {
        START(0), LEVEL_ONE(9), LEVEL_TWO(25), LEVEL_THREE(50), MAX(59);

        ElevatorLevel(double distance) {
            this.distance = distance;
        }

        private final double distance;

        public double getDistance() {
            return distance;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initAutonomous(hardwareMap);
        waitForStart();
        run();
    }

    protected abstract void run();

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
     * @param level specified level
     */
    protected void useElevatorAsync(@NonNull ElevatorLevel level) {
        int targetPos = (int) (level.getDistance() * Constants.ELEVATOR_TICKS_PER_CENTIMETRE);
        robotHardware.getElevatorMotor().setTargetPosition(targetPos);
        robotHardware.getElevatorMotor().setPower(1);
        robotHardware.getElevatorMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void useClaws(boolean closeClaws) {
        if (!isStopRequested()) {
            if (closeClaws) {
                robotHardware.getRightClaw().setPosition(Constants.CLAWS_CLOSED);
                robotHardware.getLeftClaw().setPosition(Constants.CLAWS_CLOSED);
            } else {
                robotHardware.getLeftClaw().setPosition(Constants.CLAWS_OPENED);
                robotHardware.getRightClaw().setPosition(Constants.CLAWS_OPENED);
            }
        }
    }


    /**
     * Check if the elevator is busy
     * @return whether the elevator has done retreating or advancing to the required position
     */
    protected boolean isElevatorBusy(){
        return robotHardware.getElevatorMotor().isBusy();
    }
}
