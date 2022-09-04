using System.Collections.Generic;
using UnityEngine;

public static class Vector3Extensions {

    // element-wise multiply
    public static Vector3 Multiply( this Vector3 vector, Vector3 other ) {
        return new Vector3( vector.x * other.x, vector.y * other.y, vector.z * other.z );
    }

    // element-wise divide
    public static Vector3 Divide( this Vector3 vector, Vector3 other ) {
        return new Vector3( vector.x / other.x, vector.y / other.y, vector.z / other.z );
    }

    // element-wise absolute value
    public static Vector3 Absolute( this Vector3 vector ) {
        return new Vector3( Mathf.Abs( vector.x ), Mathf.Abs( vector.y ), Mathf.Abs( vector.z ) );
    }

}

public static class LINQExtensions {

    public static IEnumerable<T> Flatten<T>( this T[,] array ) {
        for( var x = 0; x < array.GetLength( 0 ); x++ ) {
            for( var y = 0; y < array.GetLength( 1 ); y++ ) {
                yield return array[x, y];
            }
        }
    }

    public static IEnumerable<T> Flatten<T>( this T[,,] array ) {
        for( var x = 0; x < array.GetLength( 0 ); x++ ) {
            for( var y = 0; y < array.GetLength( 1 ); y++ ) {
                for( var z = 0; z < array.GetLength( 2 ); z++ ) {
                    yield return array[x, y, z];
                }
            }
        }
    }

}