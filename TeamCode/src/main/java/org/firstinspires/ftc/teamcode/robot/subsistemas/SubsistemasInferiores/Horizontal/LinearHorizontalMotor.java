package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
//import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.Sensors.SensorCor;
import org.firstinspires.ftc.teamcode.robot.Sensors.SensorToque;

//@Photon
@Config
@Deprecated
public class LinearHorizontalMotor {
    public ElapsedTime tempoIndoAteOsetPoint = new ElapsedTime();
    public DcMotorEx motorHorizontal;
    public SensorToque sensorToqueHorizontal;
    public static double delayRetrair = 0.2;
    PIDController controller = new PIDController(p, i, d);
    public static boolean monitor = true, needToHold = false;
    public SensorCor colorSensor;
    public boolean isBusy;
    public static double p = 0.0040, i = 0, d = 0.000,f = 0, ll = 0, kll = 0, valorSurtoCorrente = 1.3,
            limiarDistance = 2.7;

    public int position;
    public static int margem = 200, margemAut = 20 , sense = 24;
    public static int targetPosition = 10, ID = targetPosition, minPos = -100, maxPos = 1250;
    // -800 , 1450
    public LinearHorizontalStates linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
    public LinearHorizontalMotor(HardwareMap hardwareMap) {
        this.motorHorizontal = hardwareMap.get(DcMotorEx.class, HardwareNames.horizontalInferiorMotor);
        sensorToqueHorizontal = new SensorToque(hardwareMap);
        //this.motorHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        this.position = motorHorizontal.getCurrentPosition();
        colorSensor = new SensorCor(hardwareMap, HardwareNames.colorSensor2);
        this.targetPosition = this.position;
        reset();

    }
    public double PID() {

        double kp = p;
        int linearpos = motorHorizontal.getCurrentPosition();

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);

        if(motorHorizontal.getCurrent(CurrentUnit.AMPS) > valorSurtoCorrente && targetPosition > 350 && motorHorizontal.getCurrentPosition() > 350) {
            targetPosition -= 20;
        }
        motorHorizontal.setPower(pid);
        return pid ;
    }//todo não testado
    public Action turnOnHold() {
        return new InstantAction(() -> needToHold = true);
    }

    public Action turnOffHold() {
        return new InstantAction(() -> needToHold = false);
    }
    public Action hold() {
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            boolean started = false,condicaoDeParada = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    started = true;
                }
                PID();

                condicaoDeParada = needToHold;

                if(condicaoDeParada){
                    return false;
                }

                return true;
            }
        };
    }//todo não testado
    public Action horizontalGoTo(int alvo){
        return new Action() {
            ElapsedTime timeInReset = new ElapsedTime();
            boolean passouPeloSensor = false;
            boolean started = false, condicaoDeParadaEsticou = false, condicaoDeParadaSobreposicaoAcao = false, condicaoDeParadaSurtoCorrente = false, condicaoDeParadaColorRange = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = alvo;
                    timeInReset.reset();
                    started = true;
                    isBusy = true;
                    ID = alvo;
                }

                PID();

                condicaoDeParadaEsticou = (motorHorizontal.getCurrentPosition() >= targetPosition - margem && motorHorizontal.getCurrentPosition() <= targetPosition + margem) && targetPosition > 100;
                //condicaoDeParadaSurtoCorrente = motorHorizontal.getCurrent(CurrentUnit.AMPS) > valorSurtoCorrente;
                condicaoDeParadaColorRange = (sensorIsDetectingHorintontal() && targetPosition < 1) ;
                condicaoDeParadaSobreposicaoAcao = false; // DESABILITADA

                if(condicaoDeParadaColorRange) {
                    passouPeloSensor = true;
                }

                if (!passouPeloSensor) {
                    timeInReset.reset();
                }

                // condicao para stop por conta do id
                if(ID != alvo){
                    isBusy = false;

                    return false;
                }

                if(condicaoDeParadaEsticou){
                    isBusy = false;
                    if(targetPosition > 100){
                        linearHorizontalInferiorState = LinearHorizontalStates.EXTENDED;
                    }
                    return false;
                }

                if(condicaoDeParadaColorRange && timeInReset.time() > delayRetrair){
                    isBusy = false;
                    linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
                    reset();
                    return false;
                }
               // if(condicaoDeParadaSurtoCorrente &&  alvo < 1) {
                 //   reset();
                  //  isBusy = false;
                 //   linearHorizontalInferiorState = LinearHorizontalStates.RETRACTED;
                    //return false;
               // }

                if(condicaoDeParadaSobreposicaoAcao) {
                    return false;
                }

                return true;
            }
        };
    }//todo não testado

    public boolean sensorIsDetectingHorintontal() {
        boolean condition = false;
        condition = colorSensor.getDistance() < limiarDistance;
        return  condition;
    }

    public Action goToExtended(){

        return horizontalGoTo(maxPos);
    }
    public Action goToRetracted(){

        return horizontalGoTo(minPos);
    }
    public void upSetPoint() {
        changeTarget(targetPosition + sense);
    }
    public void downSetPoint() {
        changeTarget(targetPosition - sense);
    }
    public void changeTarget(int target) {

        targetPosition = Range.clip(target,-100,160);
        tempoIndoAteOsetPoint.reset();
    }

    public void reset(){
        this.motorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 25;
        this.motorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void monitor(Telemetry telemetry,String hozizontal) {//todo okey
        if (monitor) {
            telemetry.addData("HORIZONTAL Posição do Motor: ",motorHorizontal.getCurrentPosition());
            telemetry.addData("HORIZONTAL Posição alvo: ",targetPosition);
            telemetry.addData("HORIZONTAL setPower",motorHorizontal.getPower());
            telemetry.addData("HORIZONTAL Sensor de Toque isPressed", sensorToqueHorizontal.isPressed());
            telemetry.addData("HORIZONTAL porta", motorHorizontal.getPortNumber());
            telemetry.addData("HORIZONTAL corrente",motorHorizontal.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("HORIZONTAL Estado Atual", linearHorizontalInferiorState);
            telemetry.addData("HORIZONTAL Isbusy",isBusy);
            telemetry.addData("Distancia horizontal", colorSensor.getDistance());
            telemetry.addData("Horizontal ta sendo visto", sensorIsDetectingHorintontal());

        }
    }




}