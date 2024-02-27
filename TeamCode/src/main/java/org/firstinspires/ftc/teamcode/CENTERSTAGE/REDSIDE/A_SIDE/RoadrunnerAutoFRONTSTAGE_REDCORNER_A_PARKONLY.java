package org.firstinspires.ftc.teamcode.CENTERSTAGE.REDSIDE.A_SIDE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CENTERSTAGE.HuskyLensClass;
import org.firstinspires.ftc.teamcode.CENTERSTAGE.RobotFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RED FS-A CORNER PARK ONLY", group="Linear Opmode")
public class RoadrunnerAutoFRONTSTAGE_REDCORNER_A_PARKONLY extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotFunctions robot = new RobotFunctions(hardwareMap);
        HuskyLensClass husky = new HuskyLensClass();
        husky.init(hardwareMap,"red");

        int pixelPosition = 0;// = 4;//husky.runHusky();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(62, -36, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //Place Pixel position = 3
        Trajectory leftSpike = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(40,-46.5,Math.toRadians(0)))
                .build();
        //Move forward a bit
        Trajectory positionThreeForward = drive.trajectoryBuilder(leftSpike.end(), false)
                .forward(15)
                .build();
        //Go into red wing
        Trajectory positionThreeToRightCorner = drive.trajectoryBuilder(positionThreeForward.end(), false)
                .lineToSplineHeading(new Pose2d(60,-62,Math.toRadians(90)))
                .build();
        //Go into backstage corner
        Trajectory positionThreeCorner = drive.trajectoryBuilder(positionThreeToRightCorner.end(), false)
                .lineToSplineHeading(new Pose2d(62,50,Math.toRadians(90)))
                .build();





        //Place Pixel position = 2
        Trajectory centerSpike = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(32.5,-38,Math.toRadians(0)))
                .build();
        //Move forward a bit
        Trajectory positionTwoForward = drive.trajectoryBuilder(centerSpike.end(), false)
                .forward(15)
                .build();
        //Go into red wing
        Trajectory positionTwoToRightCorner = drive.trajectoryBuilder(positionTwoForward.end(), false)
                .lineToSplineHeading(new Pose2d(60,-62,Math.toRadians(90)))
                .build();
        //Go into backstage corner
        Trajectory positionTwoCorner = drive.trajectoryBuilder(positionTwoToRightCorner.end(), false)
                .lineToSplineHeading(new Pose2d(62,50,Math.toRadians(90)))
                .build();





        //Place Pixel position = 1
        Trajectory rightSpike = drive.trajectoryBuilder(leftSpike.end(), false)
                .lineToSplineHeading(new Pose2d(32,-32,Math.toRadians(-90)))
                .build();
        //Move forward a bit
        Trajectory positionOneForward = drive.trajectoryBuilder(rightSpike.end(), false)
                .forward(15)
                .build();
        //Go into red wing
        Trajectory positionOneToRightCorner = drive.trajectoryBuilder(positionOneForward.end(), false)
                .lineToSplineHeading(new Pose2d(60,-62,Math.toRadians(90)))
                .build();
        //Go into backstage corner
        Trajectory positionOneCorner = drive.trajectoryBuilder(positionOneToRightCorner.end(), false)
                .lineToSplineHeading(new Pose2d(62,50,Math.toRadians(90)))
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
            //RIGHT SPIKE
            case 3:
                robot.liftSlide(.5, 350, 1000);
                drive.followTrajectory(leftSpike);
                robot.liftSlide(.5,0,1000);
                robot.rotateArm(0,1000);
                drive.followTrajectory(rightSpike);
                robot.backClawClose(false,1000);
                drive.followTrajectory(positionOneForward);
                drive.followTrajectory(positionOneToRightCorner);
                sleep(6000);
                drive.followTrajectory(positionOneCorner);
                break;
            //CENTER SPIKE
            case 2:
                robot.liftSlide(.5, 350, 1000);
                drive.followTrajectory(centerSpike);
                robot.backClawClose(false,1000);
                robot.liftSlide(.5,0,1000);
                robot.rotateArm(0,1000);
                drive.followTrajectory(positionTwoForward);
                drive.followTrajectory(positionTwoToRightCorner);
                sleep(6000);
                drive.followTrajectory(positionTwoCorner);
                break;
            //LEFT SPIKE
            case 1:
                robot.liftSlide(.5, 350, 1000);
                drive.followTrajectory(leftSpike);
                robot.backClawClose(false,1000);
                robot.liftSlide(.5,0,1000);
                robot.rotateArm(0,1000);
                drive.followTrajectory(positionThreeForward);
                drive.followTrajectory(positionThreeToRightCorner);
                sleep(6000);
                drive.followTrajectory(positionThreeCorner);
                break;
            //TEST CASE
            case 4:
                break;
            //IF IT IS 0 (This should not run ever)
            default:
                break;
        }


    }
}