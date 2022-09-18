using UnityEngine;

public class Flashlight : MonoBehaviour {

    public enum Mode {
        Off,
        On
    }

    public Light lightSource;

    public Mode mode = Mode.Off;

    void Start( ) {
        Debug.Assert( this.lightSource != null );

        this.lightSource.enabled = this.mode == Mode.On;
    }

}
