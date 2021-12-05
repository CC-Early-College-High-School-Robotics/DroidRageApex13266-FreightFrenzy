package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.DuckDetection;

import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MechanumDriveRoadRunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Old Blue Carousel Roadrunner Path", group="Roadrunner Paths")
public class BlueCaroselOld extends LinearOpMode {
    @Override
    public void runOpMode() {
        MechanumDriveRoadRunner drive = new MechanumDriveRoadRunner(hardwareMap);
        Devices robot = new Devices();
        robot.init(hardwareMap);

        double armHeight = 0.27;
        double backAmount = 0;
        double intakeAmount = 0;


        /* Open CV */

        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize OpenCvWebcam
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        // Open the Camera Device Asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start Camera Streaming

                // NOTE: this must be called *before* you call startStreaming(...)
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                // Start camera stream with 1280x720 resolution
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(new DuckDetection());
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });

        // camera dection print
        telemetry.addData("Duck position", new DuckDetection().getAnalysis());
        telemetry.addData("hi", "hi");

        if (new DuckDetection().getAnalysis().equals(DuckDetection.DuckPosition.RIGHT)) {
            armHeight = robot.ARM_HIGH_POS;
            backAmount = 33;
            intakeAmount = 20;
        }

        if (new DuckDetection().getAnalysis().equals(DuckDetection.DuckPosition.CENTER)) {
            armHeight = robot.ARM_MID_POS;
            backAmount = 33;
            intakeAmount = 11.5;
        }

        if (new DuckDetection().getAnalysis().equals(DuckDetection.DuckPosition.LEFT)) {
            armHeight = robot.ARM_LOW_POS;
            backAmount = 31;
            intakeAmount = 1;

        }









        Pose2d startPose = new Pose2d(-47, 60, Math.toRadians(270));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .turn(Math.toRadians(30)) // heading = 300
                .back(6.5)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .forward(6.5)
                .turn(Math.toRadians(60)) // heading = 0
                .back(10)
                .turn(Math.toRadians(90)) // heading = 900
                .back(27)
                .turn(Math.toRadians(90)) // heading = 180
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .back(backAmount)
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .forward(intakeAmount)
                .build();

        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(10)
                .build();

        TrajectorySequence Trajectory7 = drive.trajectorySequenceBuilder(Trajectory6.end())
                .back(10)
                .turn(Math.toRadians(90))
                .back(intakeAmount)
                .build();

        // Before start

        // Move camera
        robot.cameraServo.setPosition(0.25);

        // Lift box up
        robot.boxServo.setPosition(robot.BOX_UP);




        // On start

        waitForStart();
        telemetry.addData("hi", "hi");
        if(isStopRequested()) return;
        telemetry.addData("hi", "hi");



        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));

        // Run carousel
        robot.carouselMotor.setPower(robot.CAROUSEL_POWER);
        sleep(2500);
        robot.carouselMotor.setPower(0);

        // Run trajectory 2
        drive.followTrajectorySequence((Trajectory2));

        // Lift Arm
        robot.armMotor.setPower(robot.ARM_POWER);
        robot.setArmPosition(armHeight);
        sleep(2000);

        // Run Trajectory 3
        drive.followTrajectorySequence((Trajectory3));

        // Drop freight
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(1000);
/*
        // Run Trajectory 4
        drive.followTrajectorySequence((Trajectory4));

        // Put arm down
        robot.boxServo.setPosition(robot.BOX_INTAKE);
        robot.setArmPosition(robot.ARM_NEUTRAL_POS);



        // Run Trajectory 5
        drive.followTrajectorySequence((Trajectory5));

        // Turn on intake
        robot.intakeMotor.setVelocity(robot.INTAKE_VELOCITY);

        // Run Trajectory 6
        drive.followTrajectorySequence((Trajectory6));

        // Turn on intake
        robot.intakeMotor.setVelocity(0);

        // Lift Arm
        robot.armMotor.setPower(robot.ARM_POWER);
        robot.setArmPosition(armHeight);

        // Run Trajectory 7
        drive.followTrajectorySequence((Trajectory7));

        // Drop freight
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(1000);

        // Put arm down for TeleOp
        robot.boxServo.setPosition(robot.BOX_INTAKE);
        robot.setArmPosition(robot.ARM_NEUTRAL_POS);

 */
    }
}

