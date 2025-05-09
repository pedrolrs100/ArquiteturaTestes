package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HardwareNames {
    public static String

    /**************************************************
    *                  TeleopAliances                   *
    **************************************************/


    /**************************************************
    *                  Rodas Mortas                   *
    **************************************************/
        par0 = "rightBack",
        par1 = "rightFront",
        perp = "leftFront",
    /**************************************************
     *                  Motores Chassi                *
     **************************************************/
        rightFront =  "rightFront",
        leftFront =   "leftFront",
        rightBack =   "rightBack",
        leftBack =    "leftBack",
    /**************************************************
    *                  Sensores                       *
    **************************************************/
        distanceSensorL      = "sensorporta3",
        distanceSensorR      = "sensorPorta2",
        colorSensor1         = "I2Cporta1",
        colorSensor2         = "I2Cporta2",
        touchSensor          = "sensorToque",

    /**************************************************
    *         Visão Computacional / Cameras          *
    **************************************************/
        webcam1              =  "Webcam 1",
        limelight            =  "limelight",
    /**************************************************
    *              DcMotors Subsistemas               *
    **************************************************/   /* Motores Vertex */
        verticalR            = "verticalr",
        verticalL            = "verticall",
        bracoGarraSuperior   = "b",
        SugadorMotorInferior = "porta3e",
    /**************************************************
     *              Servo Subsistemas Pinça           *
     **************************************************/   /* Motores Vertex */

    horizontalSuperiorServo = "porta2c",
    horizontalInferiorMotor = "porta2e",
     horizontalInferiorServo = "porta5",
    bracoGarraSuperiorServo   = "porta3c",//todo esse

    bracoGarraInferiorServo = "porta0",
    rotacaoGarraInferiorServo = "porta2",

    rotacaoGarraSuperiorServo = "porta1",//todo esse

    angulacaoGarraInferiorServo = "porta5",

    angulacaoGarraSuperiorServo = "porta1c",//todo esse

    aberturaGarraInferiorServo = "porta3",

    aberturaGarraSuperiorServo = "porta4c", //todo esse

    /**************************************************
     *              Servo Subsistemas Sugar               *
     **************************************************/

    angulacaoSugarServo = "porta5",//todo esse
    alcapaoSugarServo = "porta2",//todo esse

    /**************************************************
     *                 Servo-motores                  *
     **************************************************/
    /**************************************************
     *                 TroughBore                  *
     **************************************************/
    horizontalSuperior    = "braco",
    getHorizontalInferior  = "braco";
}
