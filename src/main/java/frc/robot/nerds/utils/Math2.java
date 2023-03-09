package frc.robot.nerds.utils;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Math2 {
    
    public static double round(double num, int place) {
        if (place < 0) throw new IllegalArgumentException();
        BigDecimal bd = new BigDecimal(num);
        bd = bd.setScale((int) place, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
