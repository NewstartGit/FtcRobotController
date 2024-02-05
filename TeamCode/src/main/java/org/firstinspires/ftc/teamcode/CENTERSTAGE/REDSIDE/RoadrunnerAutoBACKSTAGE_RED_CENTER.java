package org.firstinspires.ftc.teamcode.CENTERSTAGE.REDSIDE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CENTERSTAGE.HuskyLensClass;
import org.firstinspires.ftc.teamcode.CENTERSTAGE.RobotFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RED BS CENTER", group="Linear Opmode")
public class RoadrunnerAutoBACKSTAGE_RED_CENTER extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotFunctions robot = new RobotFunctions(hardwareMap);
        HuskyLensClass husky = new HuskyLensClass();
        husky.init(hardwareMap,"red");

        int pixelPosition = 0;// = 4;//husky.runHusky();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //Place Pixel position = 3
        Trajectory leftSpike = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(34,30,Math.toRadians(90)))
                .build();
        //Place APRILTAG position = 3
        Trajectory leftAprilTag = drive.trajectoryBuilder(leftSpike.end(), false)
                .lineToSplineHeading(new Pose2d(44,49,Math.toRadians(90)))
                .build();
        //Back up position 3
        Trajectory positionOneBack = drive.trajectoryBuilder(leftAprilTag.end(), false)
                .back(10)
                .build();
        //Park in corner position 3
        Trajectory positionOneParkCenter = drive.trajectoryBuilder(positionOneBack.end(), false)
                .lineToSplineHeading(new Pose2d(12,55,Math.toRadians(180)))
                .build();


        //Place Pixel position = 2
        Trajectory centerSpike = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(24.5,18,Math.toRadians(90)))
                .build();
        //Place APRILTAG position = 2
        Trajectory centerAprilTag = drive.trajectoryBuilder(centerSpike.end(), false)
                .lineToSplineHeading(new Pose2d(36,49,Math.toRadians(90)))
                .build();
        //Back up position 2
        Trajectory positionTwoBack = drive.trajectoryBuilder(centerAprilTag.end(), false)
                .back(15)
                .build();
        //Park in corner position 2
        Trajectory positionTwoParkCenter = drive.trajectoryBuilder(positionTwoBack.end(), false)
                .lineToSplineHeading(new Pose2d(12,55,Math.toRadians(180)))
                .build();


        //Place Pixel position = 1
        Trajectory rightSpike = drive.trajectoryBuilder(leftSpike.end(), false)
                .back(22)
                .build();
        //Place APRILTAG position = 1
        Trajectory rightAprilTag = drive.trajectoryBuilder(rightSpike.end(), false)
                .lineToSplineHeading(new Pose2d(30,50,Math.toRadians(90)))
                .build();
        //Back up position 1
        Trajectory positionThreeBack = drive.trajectoryBuilder(rightAprilTag.end(), false)
                .back(15)
                .build();
        //Park in corner position 1
        Trajectory positionThreeParkCenter = drive.trajectoryBuilder(positionThreeBack.end(), false)
                .lineToSplineHeading(new Pose2d(12,55,Math.toRadians(180)))
                .build();

        Trajectory test = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(34,30,Math.toRadians(90)))
                .build();

        //Robot closes both claws and reveals the huskylens
        robot.closeClaw(true,500);
        robot.backClawClose(true, 100);

        waitForStart();
        if(isStopRequested()) return;

        robot.rotateArm(.4,1000);

        //When OpMode starts, run husky lens with the loop and leave once it returns anything other than 0
        while(pixelPosition == 0)
        {
            pixelPosition = husky.runHusky();
        }
        //pixelPosition = husky.runHusky();
        //Switch statement based on above loop
        switch(pixelPosition)
        {
            //LEFT SPIKE
            case 1:
                robot.liftSlide(.5, 350, 1000);
                drive.followTrajectory(rightSpike);
                robot.backClawClose(false,1000);
                robot.rotateArm(.75,100);
                drive.followTrajectory(rightAprilTag);
                robot.closeClaw(false,100);
                drive.followTrajectory(positionThreeBack);
                drive.followTrajectory(positionThreeParkCenter);
                robot.liftSlide(.5, 0, 1000);
                robot.rotateArm(1,100);
                break;
            //CENTER SPIKE
            case 2:
                robot.liftSlide(.5, 350, 1000);
                drive.followTrajectory(centerSpike);
                robot.backClawClose(false,1000);
                robot.rotateArm(.75,100);
                drive.followTrajectory(centerAprilTag);
                robot.closeClaw(false,100);
                drive.followTrajectory(positionTwoBack);
                drive.followTrajectory(positionTwoParkCenter);
                robot.liftSlide(.5, 0, 1000);
                robot.rotateArm(1,100);
                break;
            //RIGHT SPIKE
            case 3:
                robot.liftSlide(.5, 350, 1000);
                drive.followTrajectory(leftSpike);
                robot.backClawClose(false,1000);
                robot.rotateArm(.75,100);
                drive.followTrajectory(leftAprilTag);
                robot.closeClaw(false,100);
                drive.followTrajectory(positionOneBack);
                drive.followTrajectory(positionOneParkCenter);
                robot.liftSlide(.5, 0, 1000);
                robot.rotateArm(1,100);
                break;
            //TEST CASE
            case 4:
                drive.followTrajectory(test);
                break;
            //IF IT IS 0 (This should not run ever)
            default:
                break;
        }


    }
}