package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cannon extends SubsystemBase{
    AnalogPotentiometer transducer = new AnalogPotentiometer(0, 200,0);
    //define compressors
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Compressor pcmCompressor2 = new Compressor(1, PneumaticsModuleType.CTREPCM);

    public Cannon(){
        pcmCompressor.enableDigital();

    }

    //fill: 120 Psi
    //Shoot: 60 Psi

    public void fill(){
        //add code yayayayay

    }

}