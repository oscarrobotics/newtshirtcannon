package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase{
    //Volts --> PSI
    //Compare the analog sensor to digital readings for offset
    AnalogPotentiometer transducer = new AnalogPotentiometer(0, 5, 0);
    //define compressors
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Compressor pcmCompressor2 = new Compressor(1, PneumaticsModuleType.CTREPCM);

    //Create solenoid objects


    public Cannon(){
        pcmCompressor.enableDigital();

    }

    //fill: 120 Psi -> can't go above 125 psi
    //Shoot: 60 Psi

    //60: 1.62
    //50: 1.367
    //40: 1.196
    //30: 0.98
    //20: 0.76
    //10: 0.55
    //0: 0.5


    public void fill(){
        //Check ransducer --> dummyPSI();
        //Under 125 PSI: Set solenoid = true
        //If over: set to false
        System.out.println(dummyPSI());
    }

    public void shoot(){
        //Set solenoid = false
    }

    //getter for PSI
    public double dummyPSI(){
        return transducer.get();
    }

    //Scaling factor --> mx+b

}