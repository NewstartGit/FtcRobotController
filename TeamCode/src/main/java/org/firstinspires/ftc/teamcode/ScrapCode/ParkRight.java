package org.firstinspires.ftc.teamcode.ScrapCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MechanumClass;

@Disabled

@Autonomous(name="Park Right", group="Linear Opmode")
public class ParkRight extends LinearOpMode
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
        //aTag.initAprilTag(hardwareMap);
        imu.initIMU(hardwareMap);
        cam.init(hardwareMap);
        mc.closeClaw(true,1000);
        //tensorflow.initTfod(hardwareMap);
        waitForStart();

        if(opModeIsActive())
        {
            //mc.closeClaw(true,1000);
            //mc.drive(90,.5,3000,500,true);
            mc.drive(0,.5,5000,7750,true);
            mc.drive(180,.5,3000,500,true);
        }

    }
}
