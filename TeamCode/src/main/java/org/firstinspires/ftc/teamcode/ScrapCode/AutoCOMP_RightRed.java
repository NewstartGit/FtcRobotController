package org.firstinspires.ftc.teamcode.ScrapCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MechanumClass;

@Disabled

@Autonomous(name="((COMP)) BACKSTAGE RED", group="Linear Opmode")
public class AutoCOMP_RightRed extends LinearOpMode
{
    MechanumClass mc = new MechanumClass();
    //AprilTagClass aTag = new AprilTagClass();

    IMUClass imu = new IMUClass();
    //TensorflowClass tensorflow = new TensorflowClass();
    CameraClass cam = new CameraClass();
    @Override
    public void runOpMode() throws InterruptedException
    {
        mc.init(hardwareMap, true);

        imu.initIMU(hardwareMap);
        cam.init(hardwareMap);

        mc.backClawClose(true,100);
        mc.closeClaw(true,200);

        waitForStart();

        if(opModeIsActive()) {


            mc.closeClaw(true, 200);

            int pixelPosition = 0;

            mc.liftSlide(.5, 2400, 4500);

            //Scan
            while (opModeIsActive()) {
                if (pixelPosition == 0) {
                    //Update pixelPosition variable
                    pixelPosition = mc.returnPixelRegion(cam);
                } else {
                    break;
                }

            }

            mc.closeClaw(true, 1000);

            mc.rotateArm(.75, 100);
            mc.liftSlide(.5, 350, 2000);

            //mc.closeClaw(true, 100);
            //mc.rotateArm(1,250);

            switch (pixelPosition) {
                case 1: // Left
                    //Move right to align with pixel
                    mc.drive(0,.75,2000,1000,true);
                    //Move forward
                    mc.drive(90, .75, 1000, 1000, true);
                    //Turn to face board
                    mc.rotate(-85, .5, 4000, imu);
                    //Move to align with pixel
                    mc.drive(180, .5, 3400, 2900, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Back up into the pixel
                    mc.drive(270, .75, 1500, 1100, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Open claw
                    mc.backClawClose(false, 1);
                    //Prevent weird glitch
                    mc.drive(0, 0, 200, 0, false);
                    //Move forward
                    mc.drive(90, .75, 2500, 2000, true);
                    mc.drive(0, 0, 100, 0, false);

                    //Adjust
                    mc.rotate(10,.25,2000,imu);
                    //START TO MOVE TO PLACE PIXEL

                    //Move to left to align with apriltag
                    mc.drive(180, .75, 2000, 850, true);

                    //Move forward to place pixel
                    mc.drive(90, .5, 4200, 3800, true);
                    //Drop pixel
                    mc.drive(270, .1, 500, 30, true);
                    mc.closeClaw(false, 1);
                    mc.drive(0, 0, 100, 0, false);
                    mc.drive(270, .75, 1000, 300, true);

                    break;
                case 2: // MIDDLE
                    //Move right to align with pixel
                    mc.drive(0,.75,3000,1300,true);
                    //Move forward
                    mc.drive(90, .75, 1000, 1000, true);
                    //Move to align with pixel
                    //mc.drive(0, .5, 2000, 00, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Turn to face board (it tends to overshoot a lil bit)
                    mc.rotate(-85, .5, 4000, imu);
                    //Move to align with apriltag
                    mc.drive(180, .75, 3000, 2900, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Open claw
                    mc.backClawClose(false, 1);
                    //Prevent weird glitch where claw movement will rotate bot
                    mc.drive(0, 0, 500, 0, false);
                    //Move forward
                    mc.drive(90, .75, 1500, 500, true);
                    mc.drive(0, 0, 500, 0, false);

                    //START GOING TO PLACE PIXEL
                    //ROTATE TO CORRECT
                    mc.rotate(10,.25,2000,imu);
                    //Move right to align with center of april tags
                    mc.drive(0, .75, 3500, 1300, true);
                    mc.drive(0, 0, 100, 0, false);
                    //mc.rotate(10,.5,500,imu);
                    //Move to place pixel
                    mc.drive(90, .5, 4000, 3400, true);
                    mc.drive(0, 0, 100, 0, false);
                    //Drop pixel
                    mc.drive(270, .1, 1000, 30, true);
                    mc.closeClaw(false, 1);
                    mc.drive(0, 0, 100, 0, false);
                    mc.drive(270, .75, 1000, 200, true);
                    break;
                case 3:// RIGHT
                    //Move right
                    mc.drive(0,.75,1000,500,true);
                    //Move forward
                    mc.drive(90, .75, 1000, 1000, true);
                    //Move right to avoid hitting bar
                    //mc.drive(0,.75,1000,250,true);

                    //Turn around
                    mc.rotate(-178, .75, 4000, imu);
                    //Move to align with pixel
                    mc.drive(180, .5, 1250, 800, true);
                    //Back up into the pixel
                    mc.drive(270, .75, 1500, 1200, true);
                    mc.drive(0, 0, 100, 0, false);
                    //Open claw
                    mc.backClawClose(false, 10);
                    //Prevent weird glitch
                    mc.drive(0, 0, 100, 0, false);
                    //Move forward
                    mc.drive(90, .75, 500, 500, true);
                    //Rotate facing backboard
                    mc.rotate(105, .5, 2000, imu);

                    //START GOING TO PLACE PIXEL
                    //Move forward
                    mc.drive(90,.5,2500,2000,true);
                    //Move left
                    //mc.drive(180,.5,1500,800,true);
                    //Move forward
                    mc.drive(90, .5, 5000, 2700, true);
                    //Drop pixel
                    mc.drive(270, .1, 1000, 30, true);
                    mc.closeClaw(false, 1);
                    mc.drive(0, 0, 100, 0, false);
                    mc.drive(270, .75, 1000, 200, true);
                    break;
                default:
                    //mc.drive(180,.5,5000,5000,true);
                    break;
            }
        }


        }
}