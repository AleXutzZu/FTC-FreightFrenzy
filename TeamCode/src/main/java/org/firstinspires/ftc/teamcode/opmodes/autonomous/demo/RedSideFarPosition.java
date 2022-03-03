package org.firstinspires.ftc.teamcode.opmodes.autonomous.demo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.AutonomousControl;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "Demo: Red Side Far Position", group = "Demo Cluj-Napoca")
public class RedSideFarPosition extends AutonomousControl {

    @Override
    protected void run() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory carousel1 = drive.trajectoryBuilder(startPose).lineToLinearHeading(new Pose2d(-55, -49, Math.toRadians(45))).build();
        Trajectory carousel2 = drive.trajectoryBuilder(carousel1.end()).back(11.75).build();
        Trajectory barcode1 = drive.trajectoryBuilder(carousel2.end()).lineToLinearHeading(new Pose2d(-49, -48.6, Math.toRadians(0))).build();
        Trajectory barcode2 = drive.trajectoryBuilder(barcode1.end()).forward(8.5).build();
        Trajectory barcode1shippingHub1 = drive.trajectoryBuilder(barcode1.end()).forward(25).build();
        Trajectory barcode1shippingHub2 = drive.trajectoryBuilder(barcode1shippingHub1.end()).strafeLeft(8.8).build();
        Trajectory barcode2shippingHub1 = drive.trajectoryBuilder(barcode2.end()).forward(19).build();
        Trajectory barcode2shippingHub2 = drive.trajectoryBuilder(barcode1shippingHub2.end()).strafeLeft(2.8).build();
        Trajectory barcode1parking1 = drive.trajectoryBuilder(barcode1shippingHub2.end()).strafeRight(9.5).build();
        Trajectory barcode1parking2 = drive.trajectoryBuilder(barcode1parking1.end()).forward(62).build();
        Trajectory barcode2parking1 = drive.trajectoryBuilder(barcode2shippingHub2.end()).strafeRight(5.8).build();
        Trajectory barcode2parking2 = drive.trajectoryBuilder(barcode2parking1.end()).forward(62).build();

        drive.followTrajectory(carousel1);
        drive.followTrajectory(carousel2);
        useWheel(-1);
        sleep(5000);
        useWheel(0);
        drive.followTrajectory(barcode1);
        if (checkBarcode()){
            useElevator(Constants.ElevatorLevels.LEVEL_ONE);
            useArm(false);
            drive.followTrajectory(barcode1shippingHub1);
            drive.followTrajectory(barcode1shippingHub2);
            useClaws(false);
            drive.followTrajectory(barcode1parking1);
            useArm(true);
            useElevator(Constants.ElevatorLevels.START);
            drive.followTrajectory(barcode1parking2);
            return;
        }else{
            drive.followTrajectory(barcode2);
        }
        if (checkBarcode()){
            useElevator(Constants.ElevatorLevels.LEVEL_TWO);
        }else useElevator(Constants.ElevatorLevels.LEVEL_THREE);
        useArm(false);
        drive.followTrajectory(barcode2shippingHub1);
        drive.followTrajectory(barcode2shippingHub2);
        useClaws(false);
        sleep(1000);
        drive.followTrajectory(barcode2parking1);
        useArm(true);
        useElevator(Constants.ElevatorLevels.START);
        drive.followTrajectory(barcode2parking2);
    }
}
