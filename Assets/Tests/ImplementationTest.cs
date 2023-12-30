using System;
using System.Runtime.InteropServices;
using UnityEngine;

public abstract class ImplementationTest {

    protected const float epsilon = 3e-3f;

    protected abstract string shader { get; }

    protected float generateRandomFloat( ) {
        return UnityEngine.Random.Range( -1.0f, 1.0f );
    }

    protected Vector3 generateRandomVector2( ) {
        return new(
            this.generateRandomFloat( ),
            this.generateRandomFloat( )
        );
    }

    protected Vector3 generateRandomVector3( ) {
        return new(
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( )
        );
    }

    protected SVDQEF.SymmetricMatrix3x3 generateRandomSymmetricMatrix3x3( ) {
        return new(
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( )
        );
    }

    protected SVDQEF.Matrix3x3 generateRandomMatrix3x3( ) {
        return new(
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( ),
            this.generateRandomFloat( )
        );
    }

    protected void repeat( int times, Action test ) {
        for( var i = 0; i < times; ++i ) {
            test( );
        }
    }

    public T2 testKernel<T1, T2>( string kernelName, T1 parameter0, T2 resultZero ) {
        var solver = Resources.Load<ComputeShader>( $"Shaders/{this.shader}" );

        var kernel = solver.FindKernel( kernelName );

        var kernelParameter0Buffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T1 ) ) );
        var kernelResultBuffer     = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T2 ) ) );

        var paramater0Data = new T1[] { parameter0 };
        var resultData     = new T2[] { resultZero };

        kernelParameter0Buffer.SetData ( paramater0Data );
        kernelResultBuffer.SetData     ( resultData );

        solver.SetBuffer( kernel, $"{kernelName}Parameter0", kernelParameter0Buffer );
        solver.SetBuffer( kernel, $"{kernelName}Result",     kernelResultBuffer     );

        solver.Dispatch( kernel, 1, 1, 1 );

        kernelResultBuffer.GetData( resultData );

        kernelParameter0Buffer.Release( );
        kernelResultBuffer.Release( );

        return resultData[0];
    }

    public T3 testKernel<T1, T2, T3>( string kernelName, T1 parameter0, T2 parameter1, T3 resultZero ) {
        var solver = Resources.Load<ComputeShader>( $"Shaders/{this.shader}" );

        var kernel = solver.FindKernel( kernelName );

        var kernelParameter0Buffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T1 ) ) );
        var kernelParameter1Buffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T2 ) ) );
        var kernelResultBuffer     = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T3 ) ) );

        var paramater0Data = new T1[] { parameter0 };
        var parameter0Data = new T2[] { parameter1 };
        var resultData     = new T3[] { resultZero };

        kernelParameter0Buffer.SetData ( paramater0Data );
        kernelParameter1Buffer.SetData ( parameter0Data );
        kernelResultBuffer.SetData     ( resultData  );

        solver.SetBuffer( kernel, $"{kernelName}Parameter0", kernelParameter0Buffer );
        solver.SetBuffer( kernel, $"{kernelName}Parameter1", kernelParameter1Buffer );
        solver.SetBuffer( kernel, $"{kernelName}Result",     kernelResultBuffer     );

        solver.Dispatch( kernel, 1, 1, 1 );

        kernelResultBuffer.GetData( resultData );

        kernelParameter0Buffer.Release( );
        kernelParameter1Buffer.Release( );
        kernelResultBuffer.Release( );

        return resultData[0];
    }

    public T4 testKernel<T1, T2, T3, T4>( string kernelName, T1 parameter0, T2 parameter1, T3 parameter2, T4 resultZero ) {
        var solver = Resources.Load<ComputeShader>( $"Shaders/{this.shader}" );

        var kernel = solver.FindKernel( kernelName );

        var kernelParameter0Buffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T1 ) ) );
        var kernelParameter1Buffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T2 ) ) );
        var kernelParameter2Buffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T3 ) ) );
        var kernelResultBuffer     = new ComputeBuffer( 1, Marshal.SizeOf( typeof( T4 ) ) );

        var paramater0Data = new T1[] { parameter0 };
        var parameter0Data = new T2[] { parameter1 };
        var parameter1Data = new T3[] { parameter2 };
        var resultData     = new T4[] { resultZero };

        kernelParameter0Buffer.SetData ( paramater0Data );
        kernelParameter1Buffer.SetData ( parameter0Data );
        kernelParameter2Buffer.SetData ( parameter1Data );
        kernelResultBuffer.SetData     ( resultData  );

        solver.SetBuffer( kernel, $"{kernelName}Parameter0", kernelParameter0Buffer );
        solver.SetBuffer( kernel, $"{kernelName}Parameter1", kernelParameter1Buffer );
        solver.SetBuffer( kernel, $"{kernelName}Parameter2", kernelParameter2Buffer );
        solver.SetBuffer( kernel, $"{kernelName}Result",     kernelResultBuffer     );

        solver.Dispatch( kernel, 1, 1, 1 );

        kernelResultBuffer.GetData( resultData );

        kernelParameter0Buffer.Release( );
        kernelParameter1Buffer.Release( );
        kernelParameter2Buffer.Release( );
        kernelResultBuffer.Release( );

        return resultData[0];
    }

}