package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleFallingEdge
{
    private boolean toggleMode, toggled;

    public ToggleFallingEdge()
    {
        this.toggleMode = false;
        this.toggled = false;
    }
    /** Is given a button, only returns true on the falling edge, all other cases return false **/
    public boolean mode(boolean btn) {

        if (btn) {
            toggleMode = false;
            toggled = true;
        } else {
           if (toggled)
           {
               toggleMode = true;
               toggled = false;
           }
           else
           {
               toggleMode = false;
           }
        }
        return toggleMode;
    }

}
