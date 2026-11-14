using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using VoxelType          = SurfaceExtractor.Voxel.Type;
using Implementation     = SurfaceExtractor.Implementation;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using CPUVoxelization    = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization    = SurfaceExtractor.Implementation.GPU.Voxelization;

public partial class UniformDualContouring : Voxelizer {

    public class Corner : SurfaceExtractor.Corner {

        public struct Data {

            public Vector3       position;
            public float         density;
            public MaterialIndex materialIndex;

            public Data( SurfaceExtractor.Corner corner ) {
                this.position      = corner.position;
                this.density       = corner.density;
                this.materialIndex = corner.materialIndex;
            }

        }

        public Vector3       position      { get; }
        public float         density       { get; set; } = float.MaxValue;
        public MaterialIndex materialIndex { get; set; } = MaterialIndex.Void;

        public Corner( Vector3 position ) {
            this.position = position;
        }

        public Corner( Corner.Data cornerData ) {
            this.position      = cornerData.position;
            this.density       = cornerData.density;
            this.materialIndex = cornerData.materialIndex;
        }

        public bool Equals( SurfaceExtractor.Corner other ) {
            return this.position.Equals( other.position );
        }

        public override int GetHashCode( ) {
            return SurfaceExtractor.hashOf( this.position );
        }

    }

    public class Edge : SurfaceExtractor.Edge {

        public struct Data {

            public Vector3 intersection;
            public Vector3 normal;

            public Data( SurfaceExtractor.Edge edge ) {
                this.intersection = edge.intersection;
                this.normal       = edge.normal;
            }

        }

        public SurfaceExtractor.Corner[] corners      { get; }
        public Vector3                   intersection { get; set; }
        public Vector3                   normal       { get; set; }

        public Edge( SurfaceExtractor.Corner[] corners ) {
            this.corners = corners;
        }

        public Edge( Edge.Data edgeData, Corner.Data[] cornerData ) {
            this.corners      = new Corner[] { new( cornerData[0] ), new( cornerData[1] ) };
            this.intersection = edgeData.intersection;
            this.normal       = edgeData.normal;
        }

        public bool intersectsContour( ) {
            return (
                this.corners[0].materialIndex == MaterialIndex.Void && this.corners[1].materialIndex >= MaterialIndex.Material0
            ) || (
                this.corners[1].materialIndex == MaterialIndex.Void && this.corners[0].materialIndex >= MaterialIndex.Material0
            );
        }

        public bool Equals( SurfaceExtractor.Edge other ) {
            return this.corners[0].Equals( other.corners[0] ) && this.corners[1].Equals( other.corners[1] );
        }

        // the corners in order, as Equals compares them. The XOR of their hashes would cancel out the coordinates the
        // two corners of an edge along an axis have in common
        public override int GetHashCode( ) {
            return System.HashCode.Combine( this.corners[0].GetHashCode( ), this.corners[1].GetHashCode( ) );
        }

    }

    public class Voxel : SurfaceExtractor.Voxel {

        public struct Data {

            public Vector3 center;
            public Vector3 size;
            public Vector3 extents;
            public Vector3 minimum;
            public Vector3 maximum;
            public Vector3 vertex;
            public Vector3 normal;
            public int     index;
            public int     hasFeaturePoint;

            public Data( Voxel voxel ) {
                this.center  = voxel.center;
                this.size    = voxel.size;
                this.extents = voxel.extents;
                this.minimum = voxel.minimum;
                this.maximum = voxel.maximum;
                this.vertex  = voxel.vertex;
                this.normal  = voxel.normal;
                this.index   = voxel.index;

                this.hasFeaturePoint = Convert.ToInt32( voxel.hasFeaturePoint );
            }

        }

        public VoxelType                 type    { get; set; } = VoxelType.None;
        public Vector3                   center  { get; }
        public Vector3                   size    { get; }
        public Vector3                   extents { get; }
        public Vector3                   minimum { get; }
        public Vector3                   maximum { get; }
        public SurfaceExtractor.Corner[] corners { get; }
        public SurfaceExtractor.Edge[]   edges   { get; }
        public Vector3                   vertex  { get; set; } = Vector3.zero;
        public Vector3                   normal  { get; set; } = Vector3.zero;
        public int                       index   { get; set; } = -1;

        public bool hasFeaturePoint { get; set; } = false;

        public Voxel( Vector3 center, Vector3 size ) {
            this.center  = center;
            this.size    = size;
            this.extents = size / 2;
            this.minimum = this.center - this.extents;
            this.maximum = this.center + this.extents;

            /*
            corner layout:
                 4--------------7
                /|             /|
               / |            / |
              /  |           /  |
             5--------------6   |
             |   |          |   |
             |   |          |   |
             |   3----------|---2
             |  /           |  /
             | /            | /
             |/             |/
             0--------------1
            */
            this.corners = new Corner[] {
                // bottom
                new( this.minimum ),
                new( new Vector3( this.maximum.x, this.minimum.y, this.minimum.z ) ),
                new( new Vector3( this.maximum.x, this.minimum.y, this.maximum.z ) ),
                new( new Vector3( this.minimum.x, this.minimum.y, this.maximum.z ) ),
                // top
                new( new Vector3( this.minimum.x, this.maximum.y, this.maximum.z ) ),
                new( new Vector3( this.minimum.x, this.maximum.y, this.minimum.z ) ),
                new( new Vector3( this.maximum.x, this.maximum.y, this.minimum.z ) ),
                new( this.maximum ),
            };

            /*
            edge layout:
                 +------3-------+
                /|             /|
               10|            11|
              /  |           /  |
             +---6---2------+   7
             |   |          |   |
             |   |          |   |
             4   +------1---5---+
             |  /           |  /
             | 8            | 9
             |/             |/
             +-------0------+
            */
            this.edges = new Edge[] {
                // x axis
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[1] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[3], this.corners[2] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[6] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[4], this.corners[7] } ),
                // y axis
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[5] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[1], this.corners[6] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[3], this.corners[4] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[2], this.corners[7] } ),
                // z axis
                new( new SurfaceExtractor.Corner[] { this.corners[0], this.corners[3] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[1], this.corners[2] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[5], this.corners[4] } ),
                new( new SurfaceExtractor.Corner[] { this.corners[6], this.corners[7] } )
            };
        }

        public Voxel( Voxel.Data voxelData, Corner.Data[] cornerData, Edge.Data[] edgeData ) {
            this.center  = voxelData.center;
            this.size    = voxelData.size;
            this.extents = voxelData.size / 2;
            this.minimum = this.center - this.extents;
            this.maximum = this.center + this.extents;

            this.corners = new Corner[] {
                new( cornerData[0] ),
                new( cornerData[1] ),
                new( cornerData[2] ),
                new( cornerData[3] ),
                new( cornerData[4] ),
                new( cornerData[5] ),
                new( cornerData[6] ),
                new( cornerData[7] )
            };

            this.edges = new Edge[] {
                new( edgeData[0],  new Corner.Data[] { cornerData[0], cornerData[1] } ),
                new( edgeData[1],  new Corner.Data[] { cornerData[3], cornerData[2] } ),
                new( edgeData[2],  new Corner.Data[] { cornerData[5], cornerData[6] } ),
                new( edgeData[3],  new Corner.Data[] { cornerData[4], cornerData[7] } ),
                new( edgeData[4],  new Corner.Data[] { cornerData[0], cornerData[5] } ),
                new( edgeData[5],  new Corner.Data[] { cornerData[1], cornerData[6] } ),
                new( edgeData[6],  new Corner.Data[] { cornerData[3], cornerData[4] } ),
                new( edgeData[7],  new Corner.Data[] { cornerData[2], cornerData[7] } ),
                new( edgeData[8],  new Corner.Data[] { cornerData[0], cornerData[3] } ),
                new( edgeData[9],  new Corner.Data[] { cornerData[1], cornerData[2] } ),
                new( edgeData[10], new Corner.Data[] { cornerData[5], cornerData[4] } ),
                new( edgeData[11], new Corner.Data[] { cornerData[6], cornerData[7] } ),
            };

            this.vertex = voxelData.vertex;
            this.normal = voxelData.normal;
            this.index  = voxelData.index;

            this.hasFeaturePoint = voxelData.hasFeaturePoint != 0;
        }

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return this.center == other.center && this.size == other.size;
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    public class Quad {

        public unsafe struct Data {

            public fixed int indices[6];
            public       int subMeshIndex;

            public Data( int[] indices, int subMeshIndex ) {
                unsafe {
                    for( var indexIndex = 0; indexIndex < indices.Length; ++indexIndex ) {
                        this.indices[indexIndex] = indices[indexIndex];
                    }
                }
                this.subMeshIndex = subMeshIndex;
            }

        }

        public int[] indices      = new int[6];
        public int   subMeshIndex = -1;

        public Quad( int[] indices, int subMeshIndex ) {
            this.indices      = indices;
            this.subMeshIndex = subMeshIndex;
        }

        public Quad( Data quadData ) {
            unsafe {
                for( var indexIndex = 0; indexIndex < this.indices.Length; ++indexIndex ) {
                    this.indices[indexIndex] = quadData.indices[indexIndex];
                }
            }
            this.subMeshIndex = quadData.subMeshIndex;
        }

    }

    // each is made the first time it is used, so the voxelizer can contour without having been started
    private ManagedImplementation managedImplementation;
    private GPUImplementation     gpuImplementation;
    private BurstImplementation   burstImplementation;

    private Implementation _implementation;
    protected override Implementation implementation {
        get {
            return this.implementationType switch {
                ImplementationType.Managed => this.managedImplementation ??= new ManagedImplementation( ),
                ImplementationType.GPU     => this.gpuImplementation     ??= new GPUImplementation( ),
                ImplementationType.Burst   => this.burstImplementation   ??= new BurstImplementation( ),
                _                          => throw new Exception( "Unknown implementation type specified" )
            };
        }
        set {
            this._implementation = this.implementationType switch {
                ImplementationType.Managed => value,
                ImplementationType.GPU     => value,
                ImplementationType.Burst   => value,
                _                          => throw new Exception( "Unknown implementation type specified" ),
            };
        }
    }

    // allows skipping the auto-voxelize code for testing purposes
    [HideInInspector( )]
    public bool testing = false;

    public override void Start( ) {
        if( !testing ) {
            base.Start( );
        }
    }

    public override void release( ) {
        base.release( );
        this.gpuImplementation?.releaseBuffers( );
    }

    public override Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions ) {
        // non-power-of-2 values don't voxelize well
        var resolution = ( int )Mathf.Pow( 2, this.resolution );

        var implementation = this.implementation;

        if( implementation is BurstImplementation burstImplementation ) {
            burstImplementation.debugInformation = this.debugInformationWanted;
        }

        return implementation.voxelize(
            densityFunctions,
            resolution,
            this.minimizerIterations,
            this.binarySearchIterations,
            this.surfaceCorrectionIterations,
            this.qefSolverType,
            this.intersectionApproximation,
            this.contourNormals
        );
    }

}

[CustomEditor( typeof( UniformDualContouring ) )]
public class UniformDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );
    }

}