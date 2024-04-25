package frc.robot;

public final class Constants {

    public final static class intakeConstants{
        public static final int intake_ID = 20;
        public static final double intake_speed = 0.4;
    }
    
    public final static class passThroughConstants{
        public static final int pass_ID = 40;
        public static final double pass_speed = 0.3; 
    }

    public final static class shooterConstants{
        public static final int shooter_ID_lo = 50;
        public static final int shooter_ID_hi = 51;
        public static final double shooter_speed = 0.0;
    }
    
    public final static class pivotDownConstants {
        public static final int kMotorPort_Right = 31;
        public static final int kMotorPort_Left = 30;
    
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;
    
        public static final double kSVolts = 0.0; // 0.35
        public static final double kGVolts = 0.16; // 0.38
        public static final double kVVoltSecondPerRad = 1.8; 
        public static final double kAVoltSecondSquaredPerRad = 0.0;
    
        public static final double kMaxVelocityRadPerSecond = 3;
        public static final double kMaxAccelerationRadPerSecSquared = 10;

        public static final double kPivotOffsetRads = 0.0;
    }

    public final static class pivotUpConstants {
        public static final int kMotorPort_Right = 31;
        public static final int kMotorPort_Left = 30;
    
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    
        public static final double kSVolts = 0.0;
        public static final double kGVolts = 0.165;
        public static final double kVVoltSecondPerRad = 1; //.49
        public static final double kAVoltSecondSquaredPerRad = 0.0;
    
        public static final double kMaxVelocityRadPerSecond = 3;
        public static final double kMaxAccelerationRadPerSecSquared = 10;

        public static final double kPivotOffsetRads = 0.0;
    }
}
