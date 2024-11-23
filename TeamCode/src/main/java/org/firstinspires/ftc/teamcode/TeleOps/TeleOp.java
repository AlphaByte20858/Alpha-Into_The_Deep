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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado Dox Cria", group = "OpMode")
public class TeleOp extends OpMode {
    DcMotorEx MET, MEF, MDT, MDF, LSi, LSii, braço, motorSla; //Define os motores no sistema
    Servo yawC, garra; //Define o nome dos servos no sistema
    boolean yawG, raw;
    ElapsedTime f = new ElapsedTime(); //define contador de tempo no sistema

    //Função de init
    public void init(){
        //Definição dos motores e servos para o Drive Hub
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        yawC = hardwareMap.get(Servo.class, "yawC");
        garra = hardwareMap.get(Servo.class, "garra");
        braço = hardwareMap.get(DcMotorEx.class, "braço");
        motorSla = hardwareMap.get(DcMotorEx.class, "motorSla");

        //Define a direção dos motores
        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);
        LSi.setDirection(DcMotorSimple.Direction.REVERSE);
        braço.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSla.setDirection(DcMotorSimple.Direction.REVERSE);

        //Redefine os valores dos motores
        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Define o metodo a ser usado de contabilização dos motores
        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //define o meio de "freio" para os motores
        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        braço.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pré-definições antes de iniciar as funções
        yawC.setPosition(0);
        garra.setPosition(0.6);
        yawG = false;
        raw = false;
        f.reset();
        f.startTime();
    }

    //funções que vão se repetir, utilizadas para a partida em si e contêm os sistemas
    public void loop(){
        movi(); //função da movimentação
        linear(); //função do sistema linear
        arm(); //função do braço
        if (f.seconds() >= 1) {
            Crvos(); // função dos servos
        }
    }

    public void movi(){
        //cálculos usados para obter os valores para as mecanum
        double axial   = gamepad1.left_trigger - gamepad1.right_trigger;
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
            MotorsPower(motorEsquerdoFf * 0.8, motorDireitoFf * 0.8, motorEsquerdoTf * 0.8, motorDireitoTf * 0.8);
        }
        else {
            MotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
        }
        telemetry.addData("MDF", MDF.getCurrentPosition());
        telemetry.addData("MEF", MEF.getCurrentPosition());
        telemetry.addData("MET", MET.getCurrentPosition());
        telemetry.addData("MDT", MDT.getCurrentPosition());
    }
    public void MotorsPower(double p1, double p2, double p3,double p4) {
        MEF.setPower(p1);
        MDF.setPower(p2);
        MET.setPower(p3);
        MDT.setPower(p4);
    }

    public void Crvos(){
        //Angulação da garra
        if (gamepad2.y && !yawG){
            yawC.setPosition(0.65);
            yawG = true;
            f.reset();
        }
        else if (gamepad2.y && yawG){
            yawC.setPosition(0);
            yawG = false;
            f.reset();
        }

        //Abrir/fechar a garra
        if (gamepad2.x && !raw){
            garra.setPosition(1);
            raw = false;
            f.reset();
        }
        else if (gamepad2.x && raw) {
            garra.setPosition(0.4);
            raw = true;
            f.reset();
        }
        telemetry.addData("Garra está em:", garra.getPosition());
    }

    public void linear(){
        //funções do linear
        LSi.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        LSii.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        //motor novo implementado
        if (gamepad2.b){
            motorSla.setPower(1);
        }
        else {
            motorSla.setPower(0);
        }
    }

    public void arm(){
        //define a força do braço
        if (gamepad2.right_bumper){
            braço.setPower(0.8);
        }
        else if (gamepad2.left_bumper){
            braço.setPower(-0.8);
        }
        else {
            braço.setPower(0);
        }
    }
}