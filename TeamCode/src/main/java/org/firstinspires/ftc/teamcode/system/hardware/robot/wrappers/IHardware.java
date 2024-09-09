package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

public interface IHardware
{
    void update();
    void reads();
    void writes();
    default void emergencyStop() {}
    default void setPowers(double power){}
}
