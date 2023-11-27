package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Testing Autonomous", group="Linear Opmode")
public class CompetitionAutonomous extends LinearOpMode
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
        //aTag.initAprilTag(hardwareMap);
        imu.initIMU(hardwareMap);
        cam.init(hardwareMap);
        //tensorflow.initTfod(hardwareMap);
        waitForStart();

        if(opModeIsActive())
        {
            mc.backClawClose(true,500);

            int pixelPosition = 0;

            mc.drive(90,.5,1000,500,true);
            mc.drive(0,.5,100,0,false);

            mc.liftSlide(.5,2000,2000);

            mc.rotateArm(.95,3500);

            //Scan
            while(opModeIsActive())
            {
                if(pixelPosition == 0)
                {
                    //Update pixelPosition variable
                    pixelPosition = mc.returnPixelRegion(cam);
                }
                else
                {
                    break;
                }
                /*
                if(pixelPosition == 1)
                {
                    break;
                }



                if(pixelPosition == 2)
                {
                    break;
                }

                 */
            }

            mc.liftSlide(.5,0,2000);

            mc.rotateArm(1,250);

            switch(pixelPosition)
            {
                case 1: // Left
                    //Move forward
                    mc.drive(90,.75,1000,1000,true);
                    //Turn around
                    mc.rotate(180,.75,4000,imu);
                    //Move to align with pixel
                    mc.drive(0,.5,1000,1000,true);
                    //Back up into the pixel
                    mc.drive(270,.75,1000,1000,true);
                    mc.drive(0,0,100,0,false);
                    //Open claw
                    mc.backClawClose(false,10);
                    //Prevent weird glitch
                    mc.drive(0,0,100,0,false);
                    //Move forward
                    mc.drive(90,.75,500,500,true);
                    //Rotate facing backboard
                    mc.rotate(-90,.5,2000,imu);
                    break;
                case 2:
                    mc.drive(90,.5,2000,500,true);
                    break;
                case 3:
                    //Move forward
                    mc.drive(90,.75,1000,1000,true);
                    //Turn to face board
                    mc.rotate(90,.5,4000,imu);
                    //Move to align with pixel
                    mc.drive(0,.5,2000,2500,true);
                    //Back up into the pixel
                    mc.drive(270,.75,1000,500,true);
                    mc.drive(0,0,100,0,false);
                    //Open claw
                    mc.backClawClose(false,10);
                    //Prevent weird glitch
                    mc.drive(0,0,100,0,false);
                    //Move forward
                    mc.drive(90,.75,1500,2000,true);

                    break;
                default:
                    mc.drive(0,.5,2000,2000,true);
                    break;
            }


            //mc.drive(90,.5,2000,500,true);
            //mc.drive(90,.5,2000,500,true);
            //mc.drive(180,.5,2000,500,true);
            //mc.drive(270,.5,2000,500,true);

            //mc.rotate(90,.5,5000,imu);
            //mc.rotate(3.14/2,.5,5000,imu);

            //mc.drive(90,.5,2000,500,true);

            //mc.rotate(180,.5,5000,imu);

            //mc.drive(270,.5,2000,500,true);

            //mc.rotate(90,.5,5000,imu);

            //mc.rotate(-90,.5,5000,imu);

            /*
            int pixelPosition = 0;
            boolean driveBool = true;

            mc.liftSlide(.5,1000, 5000);

            mc.rotateArm(.9,1000);

            mc.closeClaw(true, 1000);

            mc.drive(0,.5,3000,1000,true);

            mc.rotate(90,.5,5000,imu);
            mc.rotate(-90,.5,5000,imu);

            mc.closeClaw(false, 1000);

            mc.liftSlide(.5,0,5000);


             */
        }

    }
}
