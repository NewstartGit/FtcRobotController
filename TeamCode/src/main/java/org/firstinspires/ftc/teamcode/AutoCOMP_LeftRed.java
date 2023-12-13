package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="((COMP)) FRONTSTAGE RED", group="Linear Opmode")
public class AutoCOMP_LeftRed extends LinearOpMode
{
    MechanumClass mc = new MechanumClass();
    //AprilTagClass aTag = new AprilTagClass();

    public IMUClass imu = new IMUClass();
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


            mc.closeClaw(true,200);

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

            mc.closeClaw(true,100);

            mc.rotateArm(1, 100);
            mc.liftSlide(.5, 0, 2000);

            //mc.rotateArm(1,250);

            switch (pixelPosition) {
                case 1: // Left
                    //Move forward
                    mc.drive(90, .75, 1000, 1000, true);
                    //Turn around
                    mc.rotate(180, .75, 4000, imu);
                    //Move to align with pixel
                    mc.drive(0, .5, 1250, 1250, true);
                    //Back up into the pixel
                    mc.drive(270, .75, 1000, 1000, true);
                    mc.drive(0, 0, 100, 0, false);
                    //Open claw
                    mc.backClawClose(false, 10);
                    //Prevent weird glitch
                    mc.drive(0, 0, 100, 0, false);
                    //Move forward
                    mc.drive(90, .75, 500, 500, true);
                    //Rotate facing backboard
                    mc.rotate(-178, .5, 2000, imu);

                    break;
                case 2: // MIDDLE
                    //Move forward
                    mc.drive(90, .75, 1000, 1000, true);
                    //Move to align with pixel
                    mc.drive(180, .5, 2000, 1700, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Turn to face board (it tends to overshoot a lil bit)
                    mc.rotate(95, .5, 4000, imu);
                    //Move to align with pixel
                    mc.drive(0, .75, 3000, 2900, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Open claw
                    mc.backClawClose(false, 1);
                    //Prevent weird glitch where claw movement will rotate bot
                    mc.drive(0, 0, 500, 0, false);
                    //Move forward
                    mc.drive(90, .75, 1500, 500, true);
                    mc.drive(0, 0, 500, 0, false);

                    //Rotate facing other side
                    mc.rotate(-90,.5,3000,imu);

                    break;
                case 3:// RIGHT
                    //Move forward
                    mc.drive(90, .75, 1000, 1000, true);
                    //Turn to face board
                    mc.rotate(90, .5, 4000, imu);
                    //Move to align with pixel
                    mc.drive(0, .5, 3400, 3000, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Back up into the pixel
                    mc.drive(270, .75, 1000, 750, true);
                    mc.drive(0, 0, 500, 0, false);
                    //Open claw
                    mc.backClawClose(false, 1);
                    //Prevent weird glitch
                    mc.drive(0, 0, 200, 0, false);
                    //Move forward
                    mc.drive(90, .75, 1500, 2000, true);
                    mc.drive(0, 0, 100, 0, false);

                    //Rotate facing other side
                    mc.rotate(-90,.5,3000,imu);

                    break;
                default:
                    //mc.drive(180,.5,5000,5000,true);
                    break;
            }

            boolean driveBool = true;

            //Scan for AprilTag
            /*
            while(opModeIsActive())
            {
                if(driveBool)
                {
                    driveBool = mc.alignWithAprilTag(.25,10,cam,pixelPosition);
                }
                else
                {
                    break;
                }
                //mc.drive(0,0.1,1000,1000,true);
            }

            mc.drive(90,.25,1000,100,true);

             */
        }
    }
}
