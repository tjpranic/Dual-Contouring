using UnityEngine;

public class CameraController : MonoBehaviour {

    [Min( 1 )]
    public float cameraSpeed = 50.0f;
    public float boostSpeed  = 150.0f;
    public float sensitivity = 5.0f;
    public bool  freeCamera  = false;

    private Vector2 cameraRotation = new( 0.0f, 0.0f );

    public void Start( ) {
        this.cameraRotation = Quaternion.Euler( this.transform.eulerAngles.y, this.transform.eulerAngles.x, 0.0f ).eulerAngles;
    }

    public void LateUpdate( ) {
        // x and y axis rotation
        if( Input.GetMouseButton( 0 ) ) {
            if( this.freeCamera ) {
                var x =  Input.GetAxis( "Mouse X" ) * this.sensitivity;
                var y = -Input.GetAxis( "Mouse Y" ) * this.sensitivity;

                this.transform.Rotate( 0.0f, x, 0.0f );
                this.transform.Rotate( y, 0.0f, 0.0f );
            }
            else {
                this.cameraRotation.x += Input.GetAxis( "Mouse X" ) * this.sensitivity;
                this.cameraRotation.y -= Input.GetAxis( "Mouse Y" ) * this.sensitivity;

                this.cameraRotation.x = Mathf.Repeat ( this.cameraRotation.x, 360.0f );
                this.cameraRotation.y = Mathf.Clamp  ( this.cameraRotation.y, -90.0f, 90.0f );

                this.transform.rotation = Quaternion.Euler( this.cameraRotation.y, this.cameraRotation.x, 0 );
            }
        }
        // z axis rotation, free camera only
        if( this.freeCamera && Input.GetMouseButton( 1 ) ) {
            var z = -Input.GetAxis( "Mouse X" ) * this.sensitivity;

            this.transform.Rotate( 0.0f, 0.0f, -z );
        }
        // movement
        if( Input.GetMouseButton( 0 ) || Input.GetMouseButton( 1 ) ) {
            var direction = this.getMovementDirection( );
            if( Input.GetKey( KeyCode.LeftShift ) ) {
                direction *= this.boostSpeed;
            }
            else {
                direction *= this.cameraSpeed;
            }
            direction *= Time.deltaTime;

            this.transform.Translate( direction );
        }
    }

    private Vector3 getMovementDirection( ) {
        var direction = new Vector3( 0.0f, 0.0f, 0.0f );
        if( Input.GetKey( KeyCode.W ) ) {
            direction += new Vector3( 0, 0, 1 );
        }
        if( Input.GetKey( KeyCode.S ) ) {
            direction += new Vector3( 0, 0, -1 );
        }
        if( Input.GetKey( KeyCode.A ) ) {
            direction += new Vector3( -1, 0, 0 );
        }
        if( Input.GetKey( KeyCode.D ) ) {
            direction += new Vector3( 1, 0, 0 );
        }
        if( Input.GetKey( KeyCode.Space ) ) {
            direction += new Vector3( 0, 1, 0 );
        }
        if( Input.GetKey( KeyCode.C ) ) {
            direction += new Vector3( 0, -1, 0 );
        }
        return direction.normalized;
    }

}
