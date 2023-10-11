package org.firstinspires.ftc.teamcode.drive.opmode.main.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.TagDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {
    private VoltageSensor batteryVoltageSensor;
    private TagDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        detector = new TagDetector(hardwareMap, telemetry);
        TagDetector.Tag currTag = TagDetector.Tag.noTag;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive.setPoseEstimate(new Pose2d(37, -60, Math.toRadians(90)));
        //come to high junction
        Trajectory traj1= drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(90)))
                .forward(48)
                .build();
        Trajectory traj11= drive.trajectoryBuilder(new Pose2d(37, -6, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(34, -12, Math.toRadians(135)))
                .build();
        ////score on high
        //turn to normal angle
        Trajectory traj2= drive.trajectoryBuilder(new Pose2d(37, -8, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .build();
        ////pick up cones from stack
        //back to stacked cones
        Trajectory traj3= drive.trajectoryBuilder(new Pose2d(37, -12, Math.toRadians(180)))
                .back(18)
                .build();
        ////take cone
        //move to standart position
        Trajectory traj4= drive.trajectoryBuilder(new Pose2d(55, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(34, -10, Math.toRadians(135)))
                .build();
        ////pick up cones from stack
        //move to high junction
        Trajectory traj5= drive.trajectoryBuilder(new Pose2d(37, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                .build();
        ////score on high
        Trajectory traj6= drive.trajectoryBuilder(new Pose2d(37, 12, Math.toRadians(90)))
                //.forward(52)
                .back(48)
                .build();
        Trajectory traj7= drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(90)))
                //.forward(52)
                .forward(24)
                .build();

        /**
         * Custom sleeve trajectories, 1-2-3 = left-middle-right
         */
        Trajectory traj_1= drive.trajectoryBuilder(new Pose2d(33, 0, Math.toRadians(135))) //y = 30
                .lineToLinearHeading(new Pose2d(12, -33, Math.toRadians(180)))
                .build();
        Trajectory traj_2= drive.trajectoryBuilder(new Pose2d(33, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(37, -33, Math.toRadians(90)))
                .build();
        Trajectory traj_3= drive.trajectoryBuilder(new Pose2d(33, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(64, -33, Math.toRadians(180)))
                .build();
        drive.initEverythinginStandart();
        while (!isStopRequested() && !isStarted()){
            TagDetector.Tag tag = detector.getTag();
            if(tag != TagDetector.Tag.noTag)
                currTag = tag;
            telemetry.addLine("Tag: " + tag.name());
            telemetry.update();
            sleep(40);
        }
        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(traj1);
        drive.followTrajectory(traj11);
        drive.throwCone(1, 1.5, telemetry);
        drive.lift_down();
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.take_cone();
        drive.followTrajectory(traj4);
        drive.throwCone(1, 1.5, telemetry);
        drive.lift_down();
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.throwCone(1, 1.5, telemetry);
        drive.lift_down();

        if(currTag == TagDetector.Tag.right){
            drive.followTrajectory(traj_3);
        } else if(currTag == TagDetector.Tag.mid){
            drive.followTrajectory(traj_2);
        } else {
            drive.followTrajectory(traj_1);
        }
        /*
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        new Thread(() -> drive.EXTENDIntakePosition(0.79, extd)).start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        new Thread(() -> drive.GRAB_PUTCONEIntakePosition1());
        */

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }
}