package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.LaunchSequenceCommand;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.command.TrackCommand;
import org.firstinspires.ftc.teamcode.lab.PurpleBallPipeline;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Teleop")
public class teleop extends CommandOpMode {

    private GamepadEx shooting_controller, drive_controller;
    private DriveSubsystem drive_subsystem;

    private LaunchSubsystem launchSubsystem;
    private BeltSubsystem beltSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private CameraSubsystem cameraSubsystem;

    private DistanceSensor distanceSensor;
    private PurpleBallPipeline pipeline;

    private OpenCvWebcam webcam;

    private boolean beltMovingUp = false;
    private boolean beltMovingDown = false;

    @Override
    public void initialize() {

        // ----- Ball Detection Camera (Webcam 1) -----
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new PurpleBallPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera opened.");
                webcam.startStreaming(640, 360);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // ----- Controllers -----
        drive_controller = new GamepadEx(gamepad1);
        shooting_controller = new GamepadEx(gamepad2);

        // ----- Hardware -----
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        launchSubsystem = new LaunchSubsystem(hardwareMap, telemetry);
        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        drive_subsystem = new DriveSubsystem(hardwareMap);
        cameraSubsystem = new CameraSubsystem(hardwareMap, telemetry);   // << NEW <<

        // ----- Default Commands -----
        drive_subsystem.setDefaultCommand(new DriveCommand(drive_subsystem, gamepad1));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem, shooting_controller));
        launchSubsystem.setDefaultCommand(new ShootCommand(launchSubsystem, shooting_controller));

        // -------- Belt Controls --------
        new GamepadButton(shooting_controller, GamepadKeys.Button.X)
                .whileHeld(() -> {
                    beltSubsystem.move_belt(BeltSubsystem.Direction.UTR);
                    beltMovingUp = true;
                    beltMovingDown = false;
                })
                .whenReleased(() -> {
                    beltSubsystem.stop();
                    beltMovingUp = false;
                });

        new GamepadButton(shooting_controller, GamepadKeys.Button.Y)
                .whileHeld(() -> {
                    beltSubsystem.move_belt(BeltSubsystem.Direction.DTR);
                    beltMovingDown = true;
                    beltMovingUp = false;
                })
                .whenReleased(() -> {
                    beltSubsystem.stop();
                    beltMovingDown = false;
                });

        // -------- Shoot (B) --------
//        new GamepadButton(shooting_controller, GamepadKeys.Button.B)
//                .whenPressed(new ShootCommand(launchSubsystem, gamepad2));

        // -------- Auto Launch Sequence (LB) --------
        schedule(new LaunchSequenceCommand(
                beltSubsystem,
                intakeSubsystem,
                distanceSensor,
                webcam,
                pipeline,
                () -> shooting_controller.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> gamepad2.b,
                () -> gamepad2.y,
                () -> beltMovingUp || beltMovingDown,
                telemetry
        ));

        // =======================================================
        //                AprilTag Track Toggle (X)
        // =======================================================

        TrackCommand trackCommand = new TrackCommand(cameraSubsystem, drive_subsystem, telemetry);

        new GamepadButton(drive_controller, GamepadKeys.Button.X)
                .toggleWhenPressed(trackCommand);
    }
}
