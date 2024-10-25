package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorSample extends OpMode {
    DcMotorEx MDT, MDF, MET, MEF;
    public void init() {
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");

        MDF.setDirection(DcMotorSimple.Direction.FORWARD);
        MDT.setDirection(DcMotorSimple.Direction.FORWARD);
        MET.setDirection(DcMotorSimple.Direction.FORWARD);
        MEF.setDirection(DcMotorSimple.Direction.FORWARD);
        //ports: MDF = 0; MDT = 1; MET = 2; MEF = 3.

    }
    public void loop(){
        telemetry.getCaptionValueSeparator();
        if (gamepad1.a){
            MDF.setPower(1);
            telemetry.addLine("Direita Frente ligado!");
        }
        else if (gamepad1.b){
            MDT.setPower(1);
            telemetry.addLine("Direita Tras ligado!");
        }
        else if (gamepad1.x){
            MET.setPower(1);
            telemetry.addLine("Esquerda Tras ligado!");
        }
        else if (gamepad1.y){
            MEF.setPower(1);
            telemetry.addLine("Esquerda Frente ligado!");
        }
        else {
            MDF.setPower(0);
            MDT.setPower(0);
            MET.setPower(0);
            MEF.setPower(0);
            telemetry.addLine("nada ativo");
        }
    }
}
