package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;

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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 12, Math.toRadians(160)))
                        .waitSeconds(5)
                        .splineTo(new Vector2d(36.1, 25), Math.PI/2)
                        .turn(Math.PI)
                        .waitSeconds(10)
                        .strafeTo(new Vector2d(36.1, 61.8))
                        .strafeTo(new Vector2d(52.2, 14.4))
                        .turn(Math.toRadians(-115))
                        .waitSeconds(10)
                        .build());

        Image img = null;
        //This is the absolute path/ THE ONLY PATH THAT WORKS idk gg gang 😭
        try { img = ImageIO.read(new File("/home/Ryan/Downloads/field-2025-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .addEntity(myBot);
        meepMeep.start();
    }
}