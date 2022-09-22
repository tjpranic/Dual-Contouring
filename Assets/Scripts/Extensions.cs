using System.Collections.Generic;
using UnityEngine;

public static class Vector3Extensions {

    // element-wise multiply
    public static Vector3 multiply( this Vector3 vector, Vector3 other ) {
        return new Vector3( vector.x * other.x, vector.y * other.y, vector.z * other.z );
    }

    // element-wise divide
    public static Vector3 divide( this Vector3 vector, Vector3 other ) {
        return new Vector3( vector.x / other.x, vector.y / other.y, vector.z / other.z );
    }

    // element-wise absolute value
    public static Vector3 absolute( this Vector3 vector ) {
        return new Vector3( Mathf.Abs( vector.x ), Mathf.Abs( vector.y ), Mathf.Abs( vector.z ) );
    }

}

public static class Vector4Extensions {

    public static Vector3 toVector3( this Vector4 vector ) {
        return new Vector3( vector.x, vector.y, vector.z );
    }

}

public static class Matrix4x4Extensions {

    public static Matrix4x4 add( this Matrix4x4 matrix, Matrix4x4 other ) {
        return new Matrix4x4(
            matrix.GetColumn( 0 ) + other.GetColumn( 0 ),
            matrix.GetColumn( 1 ) + other.GetColumn( 1 ),
            matrix.GetColumn( 2 ) + other.GetColumn( 2 ),
            matrix.GetColumn( 3 ) + other.GetColumn( 3 )
        );
    }

}

public static class LINQExtensions {

    public static IEnumerable<T> flatten<T>( this T[,] array ) {
        for( var x = 0; x < array.GetLength( 0 ); x++ ) {
            for( var y = 0; y < array.GetLength( 1 ); y++ ) {
                yield return array[x, y];
            }
        }
    }

    public static IEnumerable<T> flatten<T>( this T[,,] array ) {
        for( var x = 0; x < array.GetLength( 0 ); x++ ) {
            for( var y = 0; y < array.GetLength( 1 ); y++ ) {
                for( var z = 0; z < array.GetLength( 2 ); z++ ) {
                    yield return array[x, y, z];
                }
            }
        }
    }

}