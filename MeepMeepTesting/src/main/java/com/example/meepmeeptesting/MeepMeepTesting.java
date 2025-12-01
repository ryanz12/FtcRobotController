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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61.5, -12.5, Math.toRadians(-180)))
                        .strafeTo(new Vector2d(58, -12.5))
                        .turn(Math.toRadians(20))
                        .waitSeconds(1)
                        .turn(Math.toRadians(-110)) //turn to 270 deg
                        .strafeTo(new Vector2d(36, -29))
                        .strafeTo(new Vector2d(36, -62.5))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(52, -14))
                        .turn(Math.toRadians(110))
                        .waitSeconds(4)
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