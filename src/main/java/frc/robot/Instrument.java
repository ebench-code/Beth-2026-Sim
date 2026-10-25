package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

class Instrument extends Thread {
    private final CANcoder _CANCoder;

    public Instrument(CANcoder coder) {
        this._CANCoder = coder;
    }
    
    void printValue(double val, String units, double timestamp) {
        System.out.printf("%20f %-20s @ %f%n", val, units, timestamp);
    }

    public void run() {
        double posValue = _CANCoder.getPosition().getValueAsDouble();
        double posTstmp = _CANCoder.getPosition().getTimestamp().getTime();

        double absValue = _CANCoder.getAbsolutePosition().getValueAsDouble();
        double absTstmp = _CANCoder.getAbsolutePosition().getTimestamp().getTime();

        double velValue = _CANCoder.getVelocity().getValueAsDouble();
        double velTstmp = _CANCoder.getVelocity().getTimestamp().getTime();

        double batValue = _CANCoder.getSupplyVoltage().getValueAsDouble();
        double batTstmp = _CANCoder.getSupplyVoltage().getTimestamp().getTime();

        System.out.print("Position: ");
        printValue(posValue, "Rotations", posTstmp);
        System.out.print("Abs Pos : ");
        printValue(absValue, "Rotations", absTstmp);
        System.out.print("Velocity: ");
        printValue(velValue, "RPS", velTstmp);
        System.out.print("Battery : ");
        printValue(batValue, "Volts", batTstmp);

        System.out.printf("Faults Field: 0x%X%n", _CANCoder.getFaultField().getValue());
        System.out.printf("Sticky Faults Field: 0x%X%n", _CANCoder.getStickyFaultField().getValue());
        System.out.println();
    }
}
