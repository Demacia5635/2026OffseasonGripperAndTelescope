package frc.demacia.utils;

import java.util.function.BooleanSupplier;

public class DemaciaUtils{

    public static BooleanSupplier isComp = () -> false;
    public static BooleanSupplier isRed = () -> false;

    public DemaciaUtils(BooleanSupplier isComp, BooleanSupplier isRed) {
        if (DemaciaUtils.isComp == null) {
            DemaciaUtils.isComp = isComp;
        }
        if (DemaciaUtils.isRed == null) {
            DemaciaUtils.isRed = isRed;
        }
    }


    public static boolean getIsComp() {
        return DemaciaUtils.isComp.getAsBoolean();
    }

    public static boolean getIsRed() {
        return DemaciaUtils.isRed.getAsBoolean();
    }
}
