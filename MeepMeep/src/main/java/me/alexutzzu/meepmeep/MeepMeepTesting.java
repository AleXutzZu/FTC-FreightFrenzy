package me.alexutzzu.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90)))
                                .forward(15)
                                .turn(Math.toRadians(-90))
                                .addDisplacementMarker(() -> {

                                })
                                .forward(10)
                                .addDisplacementMarker(() -> {

                                })
                                .forward(14)
                                .turn(Math.toRadians(90))
                                .forward(7)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK).setDarkMode(true).setBackgroundAlpha(1).addEntity(myBot).start();
    }
}