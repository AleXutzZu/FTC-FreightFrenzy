package me.alexutzzu.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CarouselTrajectory {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(13, 17)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive ->
                        drive
                                .trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-60, -53, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-45, -48, Math.toRadians(0)))
                                .lineTo(new Vector2d(-35, -48))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL).setDarkMode(false).setBackgroundAlpha(1).addEntity(myBot).start();

    }
}
