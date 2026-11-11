using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Implementation     = SurfaceExtractor.Implementation;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;

public partial class AdaptiveDualContouring : Voxelizer {

    [Space( )]

    public bool simplification = false;

    [Min( 0.0f )]
    public float errorThreshold = 0.01f;

    [Tooltip( "Only simplify where that keeps the topology of the surface and of the regions its materials cover" )]
    public bool topologySafe = true;

    protected override Implementation implementation {
        get { throw new NotImplementedException( ); }
        set { throw new NotImplementedException( ); }
    }

    public override Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions ) {
        return this.implementationType switch {
            ImplementationType.Managed => this.voxelizeManaged( densityFunctions ),
            ImplementationType.Burst   => this.voxelizeBurst( densityFunctions ),
            ImplementationType.GPU     => throw new NotSupportedException( "Adaptive dual contouring has no GPU implementation" ),
            _                          => throw new Exception( "Unknown implementation type specified" )
        };
    }

    public override CPUVoxelization convert( GPUVoxelization voxelization ) {
        throw new NotImplementedException( );
    }

}

[CustomEditor( typeof( AdaptiveDualContouring ) )]
public class AdaptiveDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );

        var adaptiveDualContouring = this.target as AdaptiveDualContouring;

        // there is no GPU implementation of adaptive dual contouring
        if( adaptiveDualContouring.implementationType == ImplementationType.GPU ) {
            adaptiveDualContouring.implementationType = ImplementationType.Managed;
        }
    }

}