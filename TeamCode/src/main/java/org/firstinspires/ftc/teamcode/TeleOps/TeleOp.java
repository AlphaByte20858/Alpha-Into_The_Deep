package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous (name = "TeleOperado Dox Cria", group = "OpMode")
public class TeleOp extends OpMode {
    DcMotorEx MET, MEF, MDT, MDF, LS;
    Servo L, R, sexta;
    boolean Cxta = false;
    IMU imu;

    public void init() {
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        LS = hardwareMap.get(DcMotorEx.class, "LS");
        L = hardwareMap.get(Servo.class, "L");
        R = hardwareMap.get(Servo.class, "R");

        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MDT.setDirection(DcMotorSimple.Direction.REVERSE);

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }
    public void loop() {
        movi();
        Crvos();
        linear();
    }

        public void movi(){
            double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
            double lateral = gamepad1.left_stick_x * 0.9               ;
            double yaw     =  gamepad1.right_stick_x * 0.6;

            double absaxial = Math.abs(axial);
            double abslateral = Math.abs(lateral);
            double absyaw= Math.abs(yaw);
            double denominador = Math.max(absaxial + abslateral + absyaw, 1);
            YawPitchRollAngles XYZangles = imu.getRobotYawPitchRollAngles();
            AngularVelocity XYZvelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double motorEsquerdoFf = (axial + lateral + yaw / denominador);
            double motorDireitoFf = (axial - lateral - yaw / denominador);
            double motorEsquerdoTf = (axial - lateral + yaw / denominador);
            double motorDireitoTf = (axial + lateral - yaw / denominador);

            if(gamepad1.right_bumper){
                MotorsPower(motorEsquerdoFf, motorDireitoFf , motorEsquerdoTf, motorDireitoTf);
            }
            else {
                MotorsPower(motorEsquerdoFf * 0.85, motorDireitoFf * 0.85, motorEsquerdoTf * 0.85, motorDireitoTf * 0.85);
            }
            telemetry.addData("MDF", MDF.getCurrentPosition());
            telemetry.addData("MEF", MEF.getCurrentPosition());
            telemetry.addData("MET", MET.getCurrentPosition());
            telemetry.addData("MDT", MDT.getCurrentPosition());
        }
        public void MotorsPower(double p1, double p2, double p3,double p4){
            MEF.setPower(p1);
            MDF.setPower(p2);
            MET.setPower(p3);
            MDT.setPower(p4);
    }

    public void Crvos(){
        if (gamepad2.a){
            L.setPosition(L.getPosition() + 0.1);
            R.setPosition(R.getPosition() + 0.1);
        }
        else if (gamepad2.b){
            L.setPosition(L.getPosition() + 0.1);
            R.setPosition(R.getPosition() + 0.1);
        }
    }

    public void linear(){
        LS.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}
