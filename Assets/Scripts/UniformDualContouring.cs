using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEngine;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using Implementation            = SurfaceExtractor.Implementation;
using ImplementationType        = SurfaceExtractor.Implementation.Type;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VertexNormals             = SurfaceExtractor.VertexNormals;
using QEFSolverType             = QEFSolver.Type;

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

    public class GPUImplementation : Implementation {

        // constant buffer data needs to be padded so every element starts on a 4 byte aligned address
        // https://docs.microsoft.com/en-us/windows/win32/direct3dhlsl/dx-graphics-hlsl-packing-rules
        [StructLayout( LayoutKind.Explicit )]
        public struct Configuration {
            [FieldOffset( 0 * 4 )] public int resolution;
            [FieldOffset( 1 * 4 )] public int minimizerIterations;
            [FieldOffset( 2 * 4 )] public int binarySearchIterations;
            [FieldOffset( 3 * 4 )] public int surfaceCorrectionIterations;
            [FieldOffset( 4 * 4 )] public int intersectionApproximation;
            [FieldOffset( 5 * 4 )] public int vertexNormals;
            [FieldOffset( 6 * 4 )] public int densityFunctionCount;

            public Configuration(
                int                       resolution,
                int                       minimizerIterations,
                int                       binarySearchIterations,
                int                       surfaceCorrectionIterations,
                IntersectionApproximation intersectionApproximation,
                VertexNormals             vertexNormals,
                int                       densityFunctionCount
            ) {
                this.resolution                  = resolution;
                this.minimizerIterations         = minimizerIterations;
                this.binarySearchIterations      = binarySearchIterations;
                this.surfaceCorrectionIterations = surfaceCorrectionIterations;
                this.intersectionApproximation   = ( int )intersectionApproximation;
                this.vertexNormals               = ( int )vertexNormals;
                this.densityFunctionCount        = densityFunctionCount;
            }
        }

        private readonly ComputeShader uniformDualContouring;

        private ComputeBuffer configurationBuffer;
        private ComputeBuffer voxelsBuffer;
        private ComputeBuffer edgesBuffer;
        private ComputeBuffer cornersBuffer;
        private ComputeBuffer densityFunctionsBuffer;
        private ComputeBuffer vertexCountsBuffer;
        private ComputeBuffer verticesBuffer;
        private ComputeBuffer normalsBuffer;
        private ComputeBuffer quadsBuffer;
        private ComputeBuffer countsBuffer;
        private ComputeBuffer argumentsBuffer;

        private readonly int buildVoxelGridKernel;
        private readonly int sampleCornerDensitiesKernel;
        private readonly int calculateMinimizingVerticesKernel;
        private readonly int generateSharedVerticesKernel;
        private readonly int contourKernel;

        public GPUImplementation( ) {
            this.uniformDualContouring = Resources.Load<ComputeShader>( "Shaders/UniformDualContouring" );

            this.buildVoxelGridKernel              = this.uniformDualContouring.FindKernel( "buildVoxelGrid" );
            this.sampleCornerDensitiesKernel       = this.uniformDualContouring.FindKernel( "sampleCornerDensities" );
            this.calculateMinimizingVerticesKernel = this.uniformDualContouring.FindKernel( "calculateMinimizingVertices" );
            this.generateSharedVerticesKernel      = this.uniformDualContouring.FindKernel( "generateSharedVertices" );
            this.contourKernel                     = this.uniformDualContouring.FindKernel( "contour" );
        }

        public Either<CPUVoxelization, GPUVoxelization> voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            QEFSolverType                qefSolverType,
            IntersectionApproximation    intersectionApproximation,
            VertexNormals                vertexNormals
        ) {
            this.createBuffers(
                densityFunctions,
                resolution,
                minimizerIterations,
                binarySearchIterations,
                surfaceCorrectionIterations,
                intersectionApproximation,
                vertexNormals
            );

            this.buildVoxelGrid              ( resolution );
            this.sampleCornerDensities       ( resolution );
            this.calculateMinimizingVertices ( resolution );

            if( vertexNormals == VertexNormals.Shared ) {
                this.generateVertices( resolution );
            }

            this.generateIndices( resolution );

            return new Either<CPUVoxelization, GPUVoxelization>.Type1( new( ) {
                vertices  = this.verticesBuffer,
                normals   = this.normalsBuffer,
                quads     = this.quadsBuffer,
                corners   = this.cornersBuffer,
                edges     = this.edgesBuffer,
                voxels    = this.voxelsBuffer,
                counts    = this.countsBuffer,
                arguments = this.argumentsBuffer
            } );
        }

        private void buildVoxelGrid( int resolution ) {
            this.uniformDualContouring.SetBuffer ( this.buildVoxelGridKernel, "voxels",  this.voxelsBuffer );
            this.uniformDualContouring.SetBuffer ( this.buildVoxelGridKernel, "edges",   this.edgesBuffer );
            this.uniformDualContouring.SetBuffer ( this.buildVoxelGridKernel, "corners", this.cornersBuffer );
            this.uniformDualContouring.Dispatch  ( this.buildVoxelGridKernel, resolution, resolution, resolution );
        }

        private void sampleCornerDensities( int resolution ) {
            this.uniformDualContouring.SetBuffer ( this.sampleCornerDensitiesKernel, "corners",          this.cornersBuffer );
            this.uniformDualContouring.SetBuffer ( this.sampleCornerDensitiesKernel, "densityFunctions", this.densityFunctionsBuffer );
            this.uniformDualContouring.Dispatch  ( this.sampleCornerDensitiesKernel, resolution, resolution, resolution );
        }

        private void calculateMinimizingVertices( int resolution ) {
            this.uniformDualContouring.SetBuffer ( this.calculateMinimizingVerticesKernel, "voxels",           this.voxelsBuffer );
            this.uniformDualContouring.SetBuffer ( this.calculateMinimizingVerticesKernel, "edges",            this.edgesBuffer );
            this.uniformDualContouring.SetBuffer ( this.calculateMinimizingVerticesKernel, "corners",          this.cornersBuffer );
            this.uniformDualContouring.SetBuffer ( this.calculateMinimizingVerticesKernel, "densityFunctions", this.densityFunctionsBuffer );
            this.uniformDualContouring.SetBuffer ( this.calculateMinimizingVerticesKernel, "vertexCounts",     this.vertexCountsBuffer );
            this.uniformDualContouring.Dispatch  ( this.calculateMinimizingVerticesKernel, resolution, resolution, resolution );
        }

        private void generateVertices( int resolution ) {
            this.uniformDualContouring.SetBuffer ( this.generateSharedVerticesKernel, "voxels",   this.voxelsBuffer );
            this.uniformDualContouring.SetBuffer ( this.generateSharedVerticesKernel, "vertices", this.verticesBuffer );
            this.uniformDualContouring.SetBuffer ( this.generateSharedVerticesKernel, "normals",  this.normalsBuffer );
            this.uniformDualContouring.SetBuffer ( this.generateSharedVerticesKernel, "counts",   this.countsBuffer );
            this.uniformDualContouring.Dispatch  ( this.generateSharedVerticesKernel, resolution, resolution, resolution );
        }

        private void generateIndices( int resolution ) {
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "voxels",    this.voxelsBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "edges",     this.edgesBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "corners",   this.cornersBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "vertices",  this.verticesBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "normals",   this.normalsBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "quads",     this.quadsBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "counts",    this.countsBuffer );
            this.uniformDualContouring.SetBuffer ( this.contourKernel, "arguments", this.argumentsBuffer );
            this.uniformDualContouring.Dispatch  ( this.contourKernel, resolution, resolution, resolution );
        }

        private void createBuffers(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            IntersectionApproximation    intersectionApproximation,
            VertexNormals                vertexNormals
        ) {
            this.releaseBuffers( );

            var voxelCount           = resolution * resolution * resolution;
            var edgeCount            = voxelCount * SurfaceExtractor.Voxel.EdgeCount;
            var cornerCount          = voxelCount * SurfaceExtractor.Voxel.CornerCount;
            var densityFunctionCount = densityFunctions.Count( );

            var configuration = new Configuration(
                resolution,
                minimizerIterations,
                binarySearchIterations,
                surfaceCorrectionIterations,
                intersectionApproximation,
                vertexNormals,
                densityFunctionCount
            );

            var voxelData           = new Voxel.Data[voxelCount];
            var edgeData            = new Edge.Data[edgeCount];
            var cornerData          = new Corner.Data[cornerCount];
            var densityFunctionData = new DensityFunction.Data[densityFunctionCount];
            var verticesData        = new Vector3[voxelCount * 6];
            var normalsData         = new Vector3[voxelCount * 6];
            var quadsData           = new Quad.Data[voxelCount];
            var countsData          = new int[2] { 0, 0 };
            var argumentsData       = new int[4] { 0, 1, 0, 0 };

            for( var densityFunctionIndex = 0; densityFunctionIndex < densityFunctionCount; ++densityFunctionIndex ) {
                densityFunctionData[densityFunctionIndex] = new( densityFunctions.ElementAt( densityFunctionIndex ) );
            }

            // how many separate pieces of the contour pass through a voxel, for every combination of solid corners
            var vertexCountsData = new int[1 << SurfaceExtractor.Voxel.CornerCount];

            for( var solidCorners = 0; solidCorners < vertexCountsData.Length; ++solidCorners ) {
                vertexCountsData[solidCorners] = DualMarchingCubes.lookupVertexEdges( solidCorners ).Length;
            }

            this.configurationBuffer    = new ComputeBuffer( 1,                          Marshal.SizeOf( typeof( Configuration ) ), ComputeBufferType.Constant );
            this.voxelsBuffer           = new ComputeBuffer( voxelData.Length,           Marshal.SizeOf( typeof( Voxel.Data ) ) );
            this.edgesBuffer            = new ComputeBuffer( edgeData.Length,            Marshal.SizeOf( typeof( Edge.Data ) ) );
            this.cornersBuffer          = new ComputeBuffer( cornerData.Length,          Marshal.SizeOf( typeof( Corner.Data ) ) );
            this.densityFunctionsBuffer = new ComputeBuffer( densityFunctionData.Length, Marshal.SizeOf( typeof( DensityFunction.Data ) ) );
            this.vertexCountsBuffer     = new ComputeBuffer( vertexCountsData.Length,    Marshal.SizeOf( typeof( int ) ) );
            this.verticesBuffer         = new ComputeBuffer( verticesData.Length,        Marshal.SizeOf( typeof( Vector3 ) ) );
            this.normalsBuffer          = new ComputeBuffer( normalsData.Length,         Marshal.SizeOf( typeof( Vector3 ) ) );
            this.quadsBuffer            = new ComputeBuffer( quadsData.Length,           Marshal.SizeOf( typeof( Quad.Data ) ) );
            this.countsBuffer           = new ComputeBuffer( countsData.Length,          Marshal.SizeOf( typeof( int ) ) );
            this.argumentsBuffer        = new ComputeBuffer( argumentsData.Length,       Marshal.SizeOf( typeof( int ) ), ComputeBufferType.IndirectArguments );

            var configurationConstantBuffer = Shader.PropertyToID( "Configuration" );
            this.uniformDualContouring.SetConstantBuffer( configurationConstantBuffer, this.configurationBuffer, 0, Marshal.SizeOf( typeof( Configuration ) ) );

            this.configurationBuffer.SetData    ( new Configuration[] { configuration } );
            this.voxelsBuffer.SetData           ( voxelData );
            this.edgesBuffer.SetData            ( edgeData );
            this.cornersBuffer.SetData          ( cornerData );
            this.densityFunctionsBuffer.SetData ( densityFunctionData );
            this.vertexCountsBuffer.SetData     ( vertexCountsData );
            this.verticesBuffer.SetData         ( verticesData );
            this.normalsBuffer.SetData          ( normalsData );
            this.quadsBuffer.SetData            ( quadsData );
            this.countsBuffer.SetData           ( countsData );
            this.argumentsBuffer.SetData        ( argumentsData );
        }

        public void releaseBuffers( ) {
            this.configurationBuffer?.Release( );
            this.voxelsBuffer?.Release( );
            this.edgesBuffer?.Release( );
            this.cornersBuffer?.Release( );
            this.densityFunctionsBuffer?.Release( );
            this.vertexCountsBuffer?.Release( );
            this.verticesBuffer?.Release( );
            this.normalsBuffer?.Release( );
            this.quadsBuffer?.Release( );
            this.countsBuffer?.Release( );
            this.argumentsBuffer?.Release( );
        }

    }

    private ManagedImplementation managedImplementation;
    private GPUImplementation     gpuImplementation;
    private BurstImplementation   burstImplementation;

    private Implementation _implementation;
    protected override Implementation implementation {
        get {
            return this.implementationType switch {
                ImplementationType.Managed => this.managedImplementation,
                ImplementationType.GPU     => this.gpuImplementation,
                ImplementationType.Burst   => this.burstImplementation,
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
        this.managedImplementation = new ManagedImplementation( );
        this.gpuImplementation     = new GPUImplementation( );
        this.burstImplementation   = new BurstImplementation( );

        if( !testing ) {
            base.Start( );
        }
    }

    public override void OnDestroy( ) {
        this.gpuImplementation?.releaseBuffers( );
    }

    public override Either<CPUVoxelization, GPUVoxelization> voxelize( IEnumerable<DensityFunction> densityFunctions ) {
        // non-power-of-2 values don't voxelize well
        var resolution = ( int )Mathf.Pow( 2, this.resolution );

        if( this.burstImplementation != null ) {
            this.burstImplementation.debugInformation = this.debugInformationWanted;
        }

        return this.implementation.voxelize(
            densityFunctions,
            resolution,
            this.minimizerIterations,
            this.binarySearchIterations,
            this.surfaceCorrectionIterations,
            this.qefSolverType,
            this.intersectionApproximation,
            this.vertexNormals
        );
    }

    public override CPUVoxelization convert( GPUVoxelization voxelization ) {
        var resolution = ( int )Mathf.Pow( 2, this.resolution );

        var voxelCount  = resolution * resolution * resolution;
        var edgeCount   = voxelCount * SurfaceExtractor.Voxel.EdgeCount;
        var cornerCount = voxelCount * SurfaceExtractor.Voxel.CornerCount;

        var voxelData    = new Voxel.Data[voxelCount];
        var edgeData     = new Edge.Data[edgeCount];
        var cornerData   = new Corner.Data[cornerCount];
        var verticesData = new Vector3[voxelCount * 6];
        var normalsData  = new Vector3[voxelCount * 6];
        var quadsData    = new Quad.Data[voxelCount];
        var countsData   = new int[2];

        voxelization.voxels.GetData   ( voxelData );
        voxelization.edges.GetData    ( edgeData );
        voxelization.corners.GetData  ( cornerData );
        voxelization.vertices.GetData ( verticesData );
        voxelization.normals.GetData  ( normalsData );
        voxelization.quads.GetData    ( quadsData );
        voxelization.counts.GetData   ( countsData );

        // build mesh
        var quads = quadsData.Take( countsData[1] ).Select( ( quadData ) => new Quad( quadData ) );

        var mesh = SurfaceExtractor.createMesh(
            verticesData.Take ( countsData[0] ).ToArray( ),
            normalsData.Take  ( countsData[0] ).ToArray( ),
            quads.Select( ( quad ) => ( quad.subMeshIndex, quad.indices ) )
        );

        // build debug information
        var voxels = voxelData.Aggregate(
            new List<Voxel>( ),
            ( accumulator, data ) => {
                var voxelIndex       = accumulator.Count;
                var voxelCornerIndex = voxelIndex * SurfaceExtractor.Voxel.CornerCount;
                var voxelEdgeIndex   = voxelIndex * SurfaceExtractor.Voxel.EdgeCount;

                var voxelCorners = new Corner.Data[] {
                    cornerData[voxelCornerIndex + 0],
                    cornerData[voxelCornerIndex + 1],
                    cornerData[voxelCornerIndex + 2],
                    cornerData[voxelCornerIndex + 3],
                    cornerData[voxelCornerIndex + 4],
                    cornerData[voxelCornerIndex + 5],
                    cornerData[voxelCornerIndex + 6],
                    cornerData[voxelCornerIndex + 7]
                };
                var voxelEdges = new Edge.Data[] {
                    edgeData[voxelEdgeIndex + 0],
                    edgeData[voxelEdgeIndex + 1],
                    edgeData[voxelEdgeIndex + 2],
                    edgeData[voxelEdgeIndex + 3],
                    edgeData[voxelEdgeIndex + 4],
                    edgeData[voxelEdgeIndex + 5],
                    edgeData[voxelEdgeIndex + 6],
                    edgeData[voxelEdgeIndex + 7],
                    edgeData[voxelEdgeIndex + 8],
                    edgeData[voxelEdgeIndex + 9],
                    edgeData[voxelEdgeIndex + 10],
                    edgeData[voxelEdgeIndex + 11]
                };

                accumulator.Add( new( voxelData[voxelIndex], voxelCorners, voxelEdges ) );

                return accumulator;
            }
        );

        if( UnityEngine.Debug.isDebugBuild ) {
            foreach( var voxel in voxels ) {
                UnityEngine.Debug.Assert( !voxel.vertex.isNaN( ) );
                UnityEngine.Debug.Assert( !voxel.normal.isNaN( ) );
            }
        }

        var edges = voxels.Aggregate(
            new List<SurfaceExtractor.Edge>( ),
            ( accumulator, voxel ) => {
                accumulator.AddRange( voxel.edges );
                return accumulator;
            }
        ).Distinct( );

        if( UnityEngine.Debug.isDebugBuild ) {
            foreach( var edge in edges ) {
                UnityEngine.Debug.Assert( !edge.intersection.isNaN( ) );
                UnityEngine.Debug.Assert( !edge.normal.isNaN( ) );
            }
        }

        var corners = voxels.Aggregate(
            new List<SurfaceExtractor.Corner>( ),
            ( accumulator, voxel ) => {
                accumulator.AddRange( voxel.corners );
                return accumulator;
            }
        ).Distinct( );

        if( UnityEngine.Debug.isDebugBuild ) {
            foreach( var corner in corners ) {
                UnityEngine.Debug.Assert( !float.IsNaN( corner.density ) );
            }
        }

        return new CPUVoxelization {
            mesh    = mesh,
            corners = corners,
            edges   = edges,
            voxels  = voxels
        };
    }

}

[CustomEditor( typeof( UniformDualContouring ) )]
public class UniformDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );
    }

}