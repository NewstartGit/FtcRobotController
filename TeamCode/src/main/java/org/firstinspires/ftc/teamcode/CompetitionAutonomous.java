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

    HuskyLensClass husky = new HuskyLensClass();

    int pixelPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        mc.init(hardwareMap, true);
        //aTag.initAprilTag(hardwareMap);
        imu.initIMU(hardwareMap);
        cam.init(hardwareMap);
        //tensorflow.initTfod(hardwareMap);

        mc.backClawClose(true,100);
        mc.closeClaw(true,100);

        waitForStart();

        init_loop();

        if(opModeIsActive())
        {


            //int pixelPosition = 0;

            //mc.drive(90,.5,1000,500,true);
            //mc.drive(0,.5,100,0,false);

            //mc.liftSlide(.5,2400,4500);

            mc.rotateArm(.7,3500);

            //Scan
            while(opModeIsActive())
            {
                if(pixelPosition == 0)
                {
                    //Update pixelPosition variable
                    pixelPosition = husky.runHusky();
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

            //mc.rotateArm(1,250);

            switch(pixelPosition)
            {
                case 1:
                    mc.drive(0,.75,5000,2000,true);
                    break;
                case 2:
                    //Move forward
                    mc.drive(90,.75,1000,1000,true);
                    //Move to align with pixel
                    mc.drive(180,.5,2000,1700,true);
                    mc.drive(0,0,500,0,false);
                    //Turn to face board (it tends to overshoot a lil bit)
                    mc.rotate(95,.5,4000,imu);
                    //Move to align with pixel
                    mc.drive(0,.75,3000,2900,true);
                    mc.drive(0,0,500,0,false);
                    //Open claw
                    mc.backClawClose(false,1);
                    //Prevent weird glitch where claw movement will rotate bot
                    mc.drive(0,0,100,0,false);
                    //Move forward
                    mc.drive(90,.75,1500,500,true);
                    mc.drive(0,0,100,0,false);
                    //Move left to align with center of april tags
                    mc.drive(180,.75,1000,1500,true);
                    break;
                case 3:
                    //Move forward
                    mc.drive(90,.75,1000,1000,true);
                    //Turn to face board
                    mc.rotate(90,.5,4000,imu);
                    //Move to align with pixel
                    mc.drive(0,.5,2000,2500,true);
                    mc.drive(0,0,500,0,false);
                    //Back up into the pixel
                    mc.drive(270,.75,1000,750,true);
                    mc.drive(0,0,500,0,false);
                    //Open claw
                    mc.backClawClose(false,1);
                    //Prevent weird glitch
                    mc.drive(0,0,200,0,false);
                    //Move forward
                    mc.drive(90,.75,1500,2000,true);
                    mc.drive(0,0,100,0,false);

                    break;
                default:
                    //mc.drive(180,.5,5000,5000,true);
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
    //THIS NOT WORKING :((((
    @Override
    public void init_loop()
    {
        if(pixelPosition == 0)
        {
            //Update pixelPosition variable
            pixelPosition = husky.runHusky();
        }
        else
        {

        }
    }
}
