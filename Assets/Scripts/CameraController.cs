using UnityEngine;
using UnityEngine.InputSystem;

public class CameraController : MonoBehaviour {

    [Min( 1 )]
    public float cameraSpeed = 50.0f;
    public float boostSpeed  = 150.0f;
    public float sensitivity = 5.0f;
    public bool  freeCamera  = false;

    private Vector2 cameraRotation = new( 0.0f, 0.0f );

    public void Start( ) {
        cameraRotation = Quaternion.Euler( transform.eulerAngles.y, transform.eulerAngles.x, 0.0f ).eulerAngles;
    }

    public void LateUpdate( ) {
        var mouse = Mouse.current;

        // x and y axis rotation
        if( IsMouseButtonPressed( mouse, 0 ) ) {
            var mouseDelta = GetScaledMouseDelta( mouse );

            if( freeCamera ) {
                var x =  mouseDelta.x * sensitivity;
                var y = -mouseDelta.y * sensitivity;

                transform.Rotate( 0.0f, x, 0.0f );
                transform.Rotate( y, 0.0f, 0.0f );
            }
            else {
                cameraRotation.x += mouseDelta.x * sensitivity;
                cameraRotation.y -= mouseDelta.y * sensitivity;

                cameraRotation.x = Mathf.Repeat ( cameraRotation.x, 360.0f );
                cameraRotation.y = Mathf.Clamp  ( cameraRotation.y, -90.0f, 90.0f );

                transform.rotation = Quaternion.Euler( cameraRotation.y, cameraRotation.x, 0 );
            }
        }
        // z axis rotation, free camera only
        if( freeCamera && IsMouseButtonPressed( mouse, 1 ) ) {
            var z = -GetScaledMouseDelta( mouse ).x * sensitivity;

            transform.Rotate( 0.0f, 0.0f, -z );
        }
        // movement
        if( IsMouseButtonPressed( mouse, 0 ) || IsMouseButtonPressed( mouse, 1 ) ) {
            var direction = GetMovementDirection( );
            if( Keyboard.current != null && Keyboard.current.leftShiftKey.isPressed ) {
                direction *= boostSpeed;
            }
            else {
                direction *= cameraSpeed;
            }
            direction *= Time.deltaTime;

            transform.Translate( direction );
        }
    }

    private Vector3 GetMovementDirection( ) {
        var direction = new Vector3( 0.0f, 0.0f, 0.0f );
        var keyboard  = Keyboard.current;
        if( keyboard == null ) {
            return direction;
        }

        if( keyboard.wKey.isPressed ) {
            direction += new Vector3( 0, 0, 1 );
        }
        if( keyboard.sKey.isPressed ) {
            direction += new Vector3( 0, 0, -1 );
        }
        if( keyboard.aKey.isPressed ) {
            direction += new Vector3( -1, 0, 0 );
        }
        if( keyboard.dKey.isPressed ) {
            direction += new Vector3( 1, 0, 0 );
        }
        if( keyboard.spaceKey.isPressed ) {
            direction += new Vector3( 0, 1, 0 );
        }
        if( keyboard.cKey.isPressed ) {
            direction += new Vector3( 0, -1, 0 );
        }
        return direction.normalized;
    }

    private Vector2 GetScaledMouseDelta( Mouse mouse ) {
        if( mouse == null ) {
            return Vector2.zero;
        }

        // the old Input Manager "Mouse X/Y" default sensitivity is 0.1.
        return mouse.delta.ReadValue( ) * 0.1f;
    }

    private bool IsMouseButtonPressed( Mouse mouse, int mouseButton ) {
        if( mouse == null ) {
            return false;
        }

        return mouseButton switch {
            0 => mouse.leftButton.isPressed,
            1 => mouse.rightButton.isPressed,
            _ => false,
        };
    }

}
