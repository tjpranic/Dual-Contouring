using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Implementation     = SurfaceExtractor.Implementation;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;

public partial class ManifoldDualContouring : Voxelizer {

    // what a cluster's error is measured as, which decides whether it may stand in for its vertices
    public enum ErrorMeasure {
        Sum,     // the QEF at the vertex, the sum of the squared distances of the samples' planes from it, as in the paper
        Distance // the root mean square of those distances, and never less than that of the clusters below it
    }

    [Space( )]

    public ErrorMeasure errorMeasure = ErrorMeasure.Sum;

    // what the sum may come to, under Sum
    [Min( 0.0f )]
    public float errorThreshold = 6e-12f;

    // the distance a cluster's samples may be from it on average, in cells, under Distance. The sum grows with the
    // number of samples, which grows with the size of the cluster, so that a threshold of it refuses large flat
    // clusters that the small ones next to them pass. The root mean square does not grow with the size, and the
    // largest of the clusters below keeps a feature that few samples support from being averaged away. Cells, the
    // leaves, as for the simplification error, so that the same setting works at every resolution
    [Min( 0.0f )]
    public float errorDistance = 0.05f;

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

    // an octree of this depth has half as many leaves across as a grid of this resolution has cells
    public override float cellSize {
        get { return 1.0f / ( 1 << ( this.resolution - 1 ) ); }
    }

    // the distance of a cluster under Distance: the root mean square distance of the planes of its samples from its
    // vertex, given its error there, or the largest distance of the clusters among its members if that is more. The
    // vertices of the leaves have none, 0: a leaf is never judged, and with the handful of samples it has, one normal
    // taken across a sharp edge is most of its mean. At a resolution of 8 two leaves of a rotated cube came to 0.1 cells
    // that way, and every cluster above them with them. A feature that few samples support shows in the clusters of the
    // level above the leaves already: a dent of a cell over four samples is 0.4 cells in a cluster of twenty five
    private static float clusterDistance( float largestMemberDistance, float error, int intersectionCount ) {
        return Mathf.Max( largestMemberDistance, Mathf.Sqrt( error / intersectionCount ) );
    }

    // whether a cluster is close enough to its samples to stand in for them, by the measure that is set
    private bool withinError( float error, float distance ) {
        return this.errorMeasure == ErrorMeasure.Sum ? error < this.errorThreshold : distance < this.distanceThreshold;
    }

    // the error distance in object space
    private float distanceThreshold {
        get { return this.errorDistance * this.cellSize; }
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

    // the threshold of the error measure that is not set
    protected override string[] unused {
        get {
            var measure = ( ManifoldDualContouring.ErrorMeasure )this.serializedObject.FindProperty( "errorMeasure" ).enumValueIndex;

            return new[] { measure == ManifoldDualContouring.ErrorMeasure.Sum ? "errorDistance" : "errorThreshold" };
        }
    }

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );

        var manifoldDualContouring = this.target as ManifoldDualContouring;

        // there is no GPU implementation of manifold dual contouring
        if( manifoldDualContouring.implementationType == ImplementationType.GPU ) {
            manifoldDualContouring.implementationType = ImplementationType.Managed;
        }
    }

}