package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "TeleOperado Dox Cria", group = "OpMode")
public class TeleOp extends OpMode {
    DcMotorEx MET, MEF, MDT, MDF, LSi, LSii;
    Servo yawC, sR, sL, garra;
    boolean yawG, raw, braço;
    Servo sexta, ArmR, ArmL;
    boolean Cxta, Braço;

    public void init(){
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        yawC = hardwareMap.get(Servo.class, "yawC");
        sR = hardwareMap.get(Servo.class, "sR");
        sL = hardwareMap.get(Servo.class, "sL");
        garra = hardwareMap.get(Servo.class, "garra");

        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);
        sR.setDirection(Servo.Direction.REVERSE);
        sexta = hardwareMap.get(Servo.class, "sexta");
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmL = hardwareMap.get(Servo.class, "ArmL");

        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yawC.setPosition(0);
        sL.setPosition(0);
        sR.setPosition(0);
        garra.setPosition(0);

        yawG = false;
        raw = false;
        braço = false;
        sexta.setPosition(0);
        ArmR.setPosition(0);
        ArmL.setPosition(0);
        Braço = false;
    }
    public void loop(){
        movi();
        Crvos();
        linear();
    }

    public void movi(){
        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        if(gamepad1.right_bumper){
            MotorsPower(motorEsquerdoFf, motorDireitoFf , motorEsquerdoTf, motorDireitoTf);
        }
        else {
            MotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
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
        if (gamepad2.x && !yawG){
            yawC.setPosition(1);
            yawG = true;
        }
        else if (gamepad2.x && yawG){
            yawC.setPosition(0);
            yawG = false;
        }

        if (gamepad2.a && !braço){
            sR.setPosition(1);
            sL.setPosition(1);
            braço = true;
        }
        else if (gamepad2.a && braço){
            sR.setPosition(0);
            sL.setPosition(0);
            braço = false;
        }

        if (gamepad2.y && raw){
            garra.setPosition(1);
            raw = false;
        }
        else if (gamepad2.y && !raw) {
            garra.setPosition(0);
            raw = false;
        }
        else if (gamepad2.x && Cxta){
            sexta.setPosition(0);
            Cxta = false;
       }

        //braço
        if (gamepad2.right_bumper){
            if (!Braço){
                ArmR.setPosition(1);
                ArmL.setPosition(1);
            }
            else if (Braço == true){
                ArmR.setPosition(0);
                ArmL.setPosition(0);
            }
        }
    }



    public void linear(){
        LSi.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        LSii.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}