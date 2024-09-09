package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleRisingEdge
{
    private boolean toggleMode, toggled;

    public ToggleRisingEdge()
    {
        this.toggleMode = false;
        this.toggled = false;
    }
    /** Is given a button, only returns true on the rising edge, all other cases return false **/
    public boolean mode(boolean btn) {

        if (btn) {
            if (!toggled) {
                toggleMode = true; // this only runs once
                toggled = true;

            } else { // toggled is true in this case
                toggleMode = false;
            }
        } else {
            toggled = false;
            toggleMode = false;
        }
        return toggleMode;
    }

}
