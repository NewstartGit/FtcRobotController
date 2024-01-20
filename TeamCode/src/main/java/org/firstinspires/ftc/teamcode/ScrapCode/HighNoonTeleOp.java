package org.firstinspires.ftc.teamcode.ScrapCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanumClass;

@Disabled
@TeleOp(name="HighNoonTeleOp", group="Linear Opmode")
public class HighNoonTeleOp extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private MechanumClass drive = new MechanumClass();

    //private AprilTagClass aTag = new AprilTagClass();
    private CameraClass cam = new CameraClass();
    double power = .5;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.init(hardwareMap, false);

        //cam.init(hardwareMap);


        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            double horizontal = gamepad1.left_stick_y;
            double vertical = -gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            double slider = gamepad2.left_stick_y;
            double armPivot = gamepad2.right_stick_y;

            boolean intakeOpen = gamepad2.right_bumper;
            boolean intakeClose = gamepad2.left_bumper;

            boolean pivotDown = gamepad2.dpad_down;
            boolean pivotUp = gamepad2.dpad_up;
            if(gamepad1.dpad_up && power != 1)
            {
                power+=.25;
            }
            if(gamepad1.dpad_down && power != .25)
            {
                power-=.25;
            }

            //drive.teleOP(power,pivot,vertical,horizontal,slider,intakeClose,intakeOpen,pivotUp,pivotDown);

            telemetry.addData("x1 encoder val", drive.getEncoderVal("x1"));
            telemetry.addData("x2 encoder val", drive.getEncoderVal("x2"));
            telemetry.addData("y encoder val", drive.getEncoderVal("y"));
            telemetry.addData("Front Left: ", -power * pivot + (power *(-vertical - horizontal)));
            telemetry.addData("Front Right: ", power * pivot + (power *(-vertical + horizontal)));
            telemetry.addData("Back Left: ", -power * pivot + (power *(-vertical + horizontal)));
            telemetry.addData("Back Right: ", power * pivot + (power * (-vertical - horizontal)));

            //telemetry.addData("Target", aTag.returnAprilTagValues("Name"),);

            telemetry.addData("Pivot: ", pivot);
            telemetry.addData("Vertical: ", vertical);
            telemetry.addData("Horizontal: ", horizontal);
            telemetry.update();
        }


    }
}
