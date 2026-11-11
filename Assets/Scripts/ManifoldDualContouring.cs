using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Implementation     = SurfaceExtractor.Implementation;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;

public partial class ManifoldDualContouring : Voxelizer {

    [Space( )]

    [Min( 0.0f )]
    public float errorThreshold = 6e-12f;

    public bool enforceManifold = true;

    protected override Implementation implementation {
        get { throw new NotImplementedException( ); }
        set { throw new NotImplementedException( ); }
    }

    public override Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions ) {
        return this.implementationType switch {
            ImplementationType.Managed => this.voxelizeManaged( densityFunctions ),
            ImplementationType.Burst   => this.voxelizeBurst( densityFunctions ),
            ImplementationType.GPU     => throw new NotSupportedException( "Manifold dual contouring has no GPU implementation" ),
            _                          => throw new Exception( "Unknown implementation type specified" )
        };
    }

    public override CPUVoxelization convert( GPUVoxelization voxelization ) {
        throw new NotImplementedException( );
    }

    public override Mesh readMesh( GPUVoxelization voxelization ) {
        throw new NotImplementedException( );
    }

}

[CustomEditor( typeof( ManifoldDualContouring ) )]
public class ManifoldDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );

        var manifoldDualContouring = this.target as ManifoldDualContouring;

        // there is no GPU implementation of manifold dual contouring
        if( manifoldDualContouring.implementationType == ImplementationType.GPU ) {
            manifoldDualContouring.implementationType = ImplementationType.Managed;
        }
    }

}