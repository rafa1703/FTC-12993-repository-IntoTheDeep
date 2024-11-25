package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleUpOrDownWithLimit
{

    public boolean ToggleUp;
    public boolean ToggleDown;


    public int OffsetTargetPosition;
    public double UpIncrement;
    public double DownIncrement;
    public double maxOffset;

    public ToggleUpOrDownWithLimit(double upIncrement, double downIncrement, int targetPosition, int maxOffset){
        this.UpIncrement = upIncrement;
        this.DownIncrement = downIncrement;
        this.OffsetTargetPosition = targetPosition;
        this.maxOffset = maxOffset;
    }
    public void upToggle (boolean Btn){
        if (Btn) {
            if (!ToggleUp) {
                ToggleUp = true;
                // this logic runs when you press the button without releasing it
                if (OffsetTargetPosition + UpIncrement <= maxOffset)
                    OffsetTargetPosition += UpIncrement;
            }

        }
        else {
            ToggleUp = false;
        }
    }

    public void downToggle (boolean Btn, int incrementMultiplier){
        if (Btn) {
            if (!ToggleDown) {
                ToggleDown = true;
                if (OffsetTargetPosition - DownIncrement  >= 0)
                    OffsetTargetPosition -= DownIncrement * incrementMultiplier;
            }
        }
        else {
            ToggleDown = false;
        }
    }

    public void clearOffset(int target){
        OffsetTargetPosition = target;
    }

}
