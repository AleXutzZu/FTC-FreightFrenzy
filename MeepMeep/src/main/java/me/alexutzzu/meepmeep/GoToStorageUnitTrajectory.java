package me.alexutzzu.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class GoToStorageUnitTrajectory {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(10, 10)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-13, -43, Math.toRadians(90))).setReversed(true)
                        .splineToLinearHeading(new Pose2d(13, -65, 0), 0)
                        .setReversed(false)
                        .lineTo(new Vector2d(33, -65))
                        .splineToSplineHeading(new Pose2d(58, -36, Math.toRadians(-90)), 0)
                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL).addEntity(myBot).start();
    }
}
