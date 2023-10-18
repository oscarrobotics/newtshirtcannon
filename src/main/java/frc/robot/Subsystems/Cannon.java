package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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

    Solenoid shootSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Solenoid fillSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    double targetPSI=50;

    public boolean filling = true;
    


    //Create solenoid objects


    public Cannon(){
        pcmCompressor.enableDigital();
        pcmCompressor2.enableDigital();
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


    //User sets the PSI
    public void setShootPSI(double psi){
       targetPSI = psi;
    }

    //Action of filling up the tank
    public void goToPSI(){
        //Open solenoid until target PSI is reached
        //if (pressure < setPressure ) { set true } else { set false }
        if(targetPSI > psi()){
            fillSolenoid.set(true);

            filling = true;
        }
        else{
            fillSolenoid.set(false);
 
            filling = false;
        }

    }
    public boolean doneFilling(){

        return !filling;
        
    }
    public void resetfiller(){
        filling = true;
    }

    public void shoot(double trigger){
        //Set solenoid = true
        if (trigger>=0.8){
            shootSolenoid.set(true);
        }
        else{
            shootSolenoid.set(false);
        }

       
    }
    public void unshoot(){

        shootSolenoid.set(false);

    }

    //Analog values --> Digital reading (Volts --> PSI)
    /**We tested some values between the analog and digital transducer,
    /ten plotted the values and got a line of best fit. The values are linear
    so it wasn't too difficult to scale the digital value to match up (just make
    sure to divide by 4.5, or the max volt value)**/
    public double psi(){
        return (transducer.get()-0.35)*47.2;
    }

    //Scaling factor --> mx+b

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Transducer PSI", psi());
        SmartDashboard.putBoolean("Fill", fillSolenoid.get());
        SmartDashboard.putBoolean("Shoot", shootSolenoid.get());
        SmartDashboard.putNumber("Target PSI", targetPSI);
    }
} 