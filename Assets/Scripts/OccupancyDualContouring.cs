using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Implementation     = SurfaceExtractor.Implementation;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;

// Occupancy-Based Dual Contouring (Hwang and Sung, 2024): Manifold Dual Contouring on a uniform grid that only ever asks
// whether a point is inside or outside, never how far away the surface is. A point is inside where the volumes give it a
// material. There is no managed implementation
public partial class OccupancyDualContouring : Voxelizer {

    [Space( )]

    // section 5.1: where the contour crosses an edge of the grid is found by halving the edge this many times. The
    // planes the vertices are placed from go through those points, so they have to be far more exact than an
    // intersection that comes with a gradient. Past 15 the steps are smaller than a float can tell apart on a grid of
    // 256^3 cells
    [Range( 1, 20 ), Tooltip( "How many times an edge is halved to find where the contour crosses it" )]
    public int edgeSearchIterations = 15;

    // section 5.2 and appendix A.1.4: the 2D point of a face is found with three searches along rays. The first takes
    // this many samples at even steps to find where the surface is first crossed, then halves that step this many
    // times. The other two take a sample fewer and a halving more
    [Range( 1, 8 ), Tooltip( "How many samples a search across a face takes to find where it first crosses the contour" )]
    public int faceSearchSamples = 4;

    [Range( 1, 20 ), Tooltip( "How many times a search across a face then halves the step it found the contour in" )]
    public int faceSearchIterations = 11;

    // the paper's polygons are only free of intersections when every vertex is inside its own cell, and its reference
    // implementation moves a vertex that the QEF puts outside back in. That takes the vertex off the planes it was
    // placed by: an edge of a shape that passes just outside a cell is blunted there. Off, vertices stay where the QEF
    // puts them, as in manifold dual contouring, and triangles pass through each other as they do there. The split
    // below, which relies on vertices being in their cells, is then left out as well
    [Tooltip( "Move vertices that are placed outside their cell back into it. Off, edges are sharper and triangles can intersect" )]
    public bool keepVerticesInCells = true;

    // section 5.4: the polygon around an edge is split along the diagonal that keeps its triangles from intersecting
    // those of other polygons, or into four triangles around the point where the contour crosses the edge. Off, it is
    // always split along the same diagonal, as manifold dual contouring splits it
    [Tooltip( "Split each polygon so that its triangles do not intersect those of others" )]
    public bool splitQuads = true;

    // a voxelizer starts out with the managed implementation, which this one does not have. A value saved with a scene
    // is read after this and takes its place
    public OccupancyDualContouring( ) {
        this.implementationType = ImplementationType.Burst;
    }

    protected override Implementation implementation {
        get { throw new NotImplementedException( ); }
        set { throw new NotImplementedException( ); }
    }

    public override Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions ) {
        return this.implementationType switch {
            ImplementationType.Burst   => this.voxelizeBurst( densityFunctions ),
            ImplementationType.GPU     => this.voxelizeGPU( densityFunctions ),
            ImplementationType.Managed => throw new NotSupportedException( "Occupancy-based dual contouring has no managed implementation" ),
            _                          => throw new Exception( "Unknown implementation type specified" )
        };
    }

}

[CustomEditor( typeof( OccupancyDualContouring ) )]
public class OccupancyDualContouringEditor : VoxelizerEditor {

    // the settings of a voxelizer that place points by distance, which occupancy-based dual contouring never looks at:
    // it has searches of its own
    private static readonly string[] unused = {
        "m_Script",
        "_binarySearchIterations",
        "_surfaceCorrectionIterations",
        "_intersectionApproximation"
    };

    public override void OnInspectorGUI( ) {
        this.serializedObject.Update( );

        using( new EditorGUI.DisabledScope( true ) ) {
            EditorGUILayout.PropertyField( this.serializedObject.FindProperty( "m_Script" ) );
        }

        DrawPropertiesExcluding( this.serializedObject, unused );

        this.serializedObject.ApplyModifiedProperties( );

        var occupancyDualContouring = this.target as OccupancyDualContouring;

        // there is no managed implementation of occupancy-based dual contouring
        if( occupancyDualContouring.implementationType == ImplementationType.Managed ) {
            occupancyDualContouring.implementationType = ImplementationType.Burst;
        }

        // only SVD solver is available for GPU implementation
        if( occupancyDualContouring.implementationType == ImplementationType.GPU ) {
            occupancyDualContouring.qefSolverType = QEFSolver.Type.SVD;
        }
    }

}
