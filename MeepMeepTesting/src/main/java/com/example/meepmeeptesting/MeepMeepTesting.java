package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-63, 12, 0))
                        .strafeTo(new Vector2d(-60, 10))
                        .turn(Math.toRadians(-25))
                        .turn(Math.toRadians(115))
                        .strafeTo(new Vector2d(-45, 0))
                        .strafeTo(new Vector2d(-45, -24))
                        .strafeTo(new Vector2d(-60, 10))
                        .turn(Math.toRadians(-115))    .turn(Math.toRadians(115))
                        .strafeTo(new Vector2d(-25, 0))
                        .strafeTo(new Vector2d(-25, -28))
                        .strafeTo(new Vector2d(-60, 10))
                        .strafeTo(new Vector2d(25, -26))
                        .turn(Math.toRadians(-115))
//                        .waitSeconds(3)
//                        .strafeTo(new Vector2d(-11.5, 12.5))
//                        .waitSeconds(1)
//                        .turn(Math.toRadians(135))
//                        .waitSeconds(1)
//                        .strafeTo(new Vector2d(-11.5, 62.5))
//                        .waitSeconds(1)
//                        .strafeTo(new Vector2d(-11.5, 12.5))
//                        .turn(Math.toRadians(-135))
//                        .waitSeconds(1)
//                        .turn(Math.toRadians(135))
//                        .strafeTo(new Vector2d(12, 28))
//                        .strafeTo(new Vector2d(12, 62.5))
//                        .strafeTo(new Vector2d(-11.5, 12.5))
//                        .turn(Math.toRadians(-135))
//                        .strafeTo(new Vector2d(0, 48))
//                        .waitSeconds(1)
                        .build());

        Image img = null;

        //This is the absolute path/ THE ONLY PATH THAT WORKS idk gg gang 😭
        String ryan_file_path = "/home/Ryan/Downloads/field-2025-juice-light.png";
        String themika_file_path = "C:/Users/Owner/Downloads/field-2025-official.png";
        try { img = ImageIO.read(new File(ryan_file_path)); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .addEntity(myBot);
        meepMeep.start();
    }
}