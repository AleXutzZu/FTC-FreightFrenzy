package me.alexutzzu.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-35, -50, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(1.5)
                                .addTemporalMarker(() -> {
                                    //Logic to access sensor and lift elevator
                                })
                                .forward(8)
                                .waitSeconds(1.5)
                                .addTemporalMarker(() -> {
                                    //Logic to access the sensor and lift elevator to the correct level
                                })
                                .splineToSplineHeading(new Pose2d(-12, -43, Math.toRadians(90)), Math.toRadians(0))
                                .waitSeconds(3)
                                .addTemporalMarker(() -> {
                                    //Logic to bring the arm down and drop the cube
                                })
                                .strafeLeft(47)
                                .back(16)
                                .waitSeconds(6)
                                .addTemporalMarker(() -> {
                                    //Logic to rotate the carousel wheel and make a duck fall
                                })
                                .turn(Math.toRadians(-90))
                                .forward(115)
                                .addTemporalMarker(() -> {
                                    //Logic to bring down the elevator
                                })
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL).setDarkMode(false).setBackgroundAlpha(1).addEntity(myBot).start();
    }
}