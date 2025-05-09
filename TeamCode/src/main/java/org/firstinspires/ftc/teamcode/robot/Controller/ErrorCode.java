package org.firstinspires.ftc.teamcode.robot.Controller;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.HashMap;


public class ErrorCode {
    Telemetry telemetry;
    DigitalChannel ledG1,ledR1;
    DigitalChannel ledG2,ledR2;
    boolean Green = false;
    boolean Red = false;
    HashMap<Integer, String> erro = new HashMap<>();
    HashMap<Integer, Runnable> led = new HashMap<>();





    public  ErrorCode(Telemetry telemetry, HardwareMap hardwareMap) {
        this.ledG1 = hardwareMap.get(DigitalChannel.class,"LG1");
        this.ledR1 = hardwareMap.get(DigitalChannel.class,"LR1");
        this.ledG2 = hardwareMap.get(DigitalChannel.class,"LG2");
        this.ledR2 = hardwareMap.get(DigitalChannel.class,"LR2");
        this.telemetry = telemetry;
        erro.put(404,"DETECÇÃO NO LINEAR VERTICAL");
        erro.put(408,"DECTECÇÃO NO LINEAR HORIZONTAL");
        erro.put(100,"");

        led.put(404,this::trueRed);
        led.put(408, this::trueRed);
        led.put(100,this::tudoFalse);
    }

    int error;

    /*public void detecçãoDeMotoresTeste(V2 robot){
        robot.md.rightFront.setPower(0.1);
        robot.md.rightBack.setPower(0.1);
        robot.md.leftBack.setPower(0.1);
        robot.md.leftFront.setPower(0.1);
        if(robot.md.rightFront.getCurrentPosition()>100 || robot.md.rightFront.getCurrentPosition() <-100){
           telemetry.addLine("Roda Direita Frontal não responde");
        }

        if(robot.md.rightBack.getCurrentPosition()>100 || robot.md.rightBack.getCurrentPosition() <-100){
            telemetry.addLine("Roda Direita Traseira não responde");
        }

        if(robot.md.leftFront.getCurrentPosition()>100 || robot.md.leftFront.getCurrentPosition() <-100){
            telemetry.addLine("Roda Esquerda Frontal não responde");
        }

        if(robot.md.leftBack.getCurrentPosition()>100 || robot.md.leftBack.getCurrentPosition() <-100){
            telemetry.addLine("Roda Esquerda Traseira não responde");
        }
        telemetry.addLine("tudo okey");

    }*/
            /*-----------------------*/
            /*INDENTIFICAÇÃO DO ERRO */
            /*-----------------------*/
    public int  interrupLinearVertical(LinearVertical vertical){
        if(vertical.motorL.getVelocity()<5&&vertical.motorL.getCurrentPosition()>400){
            error = 404;
            return error;
        }

        error=100;
        return error;
    }
    public int  interrupLinearHorizontalSuperior(LinearHorizontalSuperior horizontal){
        /*
        if(horizontal.encoder.getVelocity()<5&&horizontal.encoder.getCurrentPosition()>400){
            error = 408;
            return error;
        }

        error=100;
        return error;

         */
        return 0;
    }


            /*---------------+/
            /*   TELEMETRIA  */
            /*---------------*/
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;

        telemetry.addLine(erro.get(error));

    }

            /*------------------*/
            /*      LED         */
            /*------------------*/

    /*public void estadosLed(){
        if(error ==404){
            Green = true;
        }
    }
    public void acenderLed(){
        if(Green){
            trueGreen();
        }
        if(Red){
            trueRed();
        }
        if(!Green){
            falseGreen();
        }
        if(!Red){
            falseRed();
        }
    }*/
    public void executableLed(){
        led.get(error).run();
    }

    public void tudoFalse(){
        falseRed();
        falseGreen();
    }

    public void trueGreen(){
        ledG1.setState(true);
        ledG2.setState(true);
    }
    public void falseGreen(){
        ledG1.setState(false);
        ledG2.setState(false);
    }
    public void trueRed(){
        ledR1.setState(true);
        ledR2.setState(true);

    }
    public void falseRed(){
        ledR1.setState(false);
        ledR2.setState(false);
    }



}
