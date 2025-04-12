package common.util.wrappers;


public abstract class MESubsystem{
    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();

}
