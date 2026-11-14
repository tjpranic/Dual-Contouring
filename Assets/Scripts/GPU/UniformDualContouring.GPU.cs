using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;

using Implementation            = SurfaceExtractor.Implementation;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using ContourVertices           = SurfaceExtractor.ContourVertices;
using QEFSolverType             = QEFSolver.Type;

public partial class UniformDualContouring {

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
                ContourVertices           contourVertices,
                int                       densityFunctionCount
            ) {
                this.resolution                  = resolution;
                this.minimizerIterations         = minimizerIterations;
                this.binarySearchIterations      = binarySearchIterations;
                this.surfaceCorrectionIterations = surfaceCorrectionIterations;
                this.intersectionApproximation   = ( int )intersectionApproximation;
                this.vertexNormals               = ( int )contourVertices;
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

        // the grid the voxel, edge, corner, vertex, normal and quad buffers were made for, -1 for none
        private int bufferResolution = -1;

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
            ContourVertices              contourVertices
        ) {
            this.prepareBuffers(
                densityFunctions,
                resolution,
                minimizerIterations,
                binarySearchIterations,
                surfaceCorrectionIterations,
                intersectionApproximation,
                contourVertices
            );

            this.buildVoxelGrid              ( resolution );
            this.sampleCornerDensities       ( resolution );
            this.calculateMinimizingVertices ( resolution );

            if( contourVertices == ContourVertices.Shared ) {
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

        // the buffers are kept from one contour to the next, and made anew only when the grid or the number of density
        // functions changes. Only what the kernels read before they write is uploaded: the configuration, the density
        // functions and the two counters. buildVoxelGrid writes every voxel, corner and edge, and the vertices, normals
        // and quads are appended. Every contour sets the constant buffer again, because all instances share the shader
        private void prepareBuffers(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            IntersectionApproximation    intersectionApproximation,
            ContourVertices              contourVertices
        ) {
            var densityFunctionData = densityFunctions.Select( ( densityFunction ) => new DensityFunction.Data( densityFunction ) ).ToArray( );

            var configuration = new Configuration(
                resolution,
                minimizerIterations,
                binarySearchIterations,
                surfaceCorrectionIterations,
                intersectionApproximation,
                contourVertices,
                densityFunctionData.Length
            );

            if( this.configurationBuffer == null ) {
                this.configurationBuffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( Configuration ) ), ComputeBufferType.Constant );
                this.countsBuffer        = new ComputeBuffer( 2, Marshal.SizeOf( typeof( int ) ) );
                this.argumentsBuffer     = new ComputeBuffer( 4, Marshal.SizeOf( typeof( int ) ), ComputeBufferType.IndirectArguments );

                // how many separate pieces of the contour pass through a voxel, for every combination of solid corners
                var vertexCountsData = new int[1 << SurfaceExtractor.Voxel.CornerCount];

                for( var solidCorners = 0; solidCorners < vertexCountsData.Length; ++solidCorners ) {
                    vertexCountsData[solidCorners] = DualMarchingCubes.lookupVertexEdges( solidCorners ).Length;
                }

                this.vertexCountsBuffer = new ComputeBuffer( vertexCountsData.Length, Marshal.SizeOf( typeof( int ) ) );
                this.vertexCountsBuffer.SetData( vertexCountsData );
            }

            if( resolution != this.bufferResolution ) {
                this.releaseGridBuffers( );

                var voxelCount = resolution * resolution * resolution;

                this.voxelsBuffer   = new ComputeBuffer( voxelCount,                                     Marshal.SizeOf( typeof( Voxel.Data ) ) );
                this.edgesBuffer    = new ComputeBuffer( voxelCount * SurfaceExtractor.Voxel.EdgeCount,   Marshal.SizeOf( typeof( Edge.Data ) ) );
                this.cornersBuffer  = new ComputeBuffer( voxelCount * SurfaceExtractor.Voxel.CornerCount, Marshal.SizeOf( typeof( Corner.Data ) ) );
                this.verticesBuffer = new ComputeBuffer( voxelCount * 6,                                 Marshal.SizeOf( typeof( Vector3 ) ) );
                this.normalsBuffer  = new ComputeBuffer( voxelCount * 6,                                 Marshal.SizeOf( typeof( Vector3 ) ) );
                this.quadsBuffer    = new ComputeBuffer( voxelCount,                                     Marshal.SizeOf( typeof( Quad.Data ) ) );

                this.bufferResolution = resolution;
            }

            // a compute buffer holds at least one element, so with no density functions it holds one that is never read
            var densityFunctionCapacity = Mathf.Max( 1, densityFunctionData.Length );

            if( this.densityFunctionsBuffer == null || this.densityFunctionsBuffer.count != densityFunctionCapacity ) {
                this.densityFunctionsBuffer?.Release( );
                this.densityFunctionsBuffer = new ComputeBuffer( densityFunctionCapacity, Marshal.SizeOf( typeof( DensityFunction.Data ) ) );
            }

            var configurationConstantBuffer = Shader.PropertyToID( "Configuration" );
            this.uniformDualContouring.SetConstantBuffer( configurationConstantBuffer, this.configurationBuffer, 0, Marshal.SizeOf( typeof( Configuration ) ) );

            this.configurationBuffer.SetData ( new Configuration[] { configuration } );
            this.countsBuffer.SetData        ( new int[] { 0, 0 } );
            this.argumentsBuffer.SetData     ( new int[] { 0, 1, 0, 0 } );

            if( densityFunctionData.Length > 0 ) {
                this.densityFunctionsBuffer.SetData( densityFunctionData );
            }
        }

        private void releaseGridBuffers( ) {
            this.voxelsBuffer?.Release( );
            this.edgesBuffer?.Release( );
            this.cornersBuffer?.Release( );
            this.verticesBuffer?.Release( );
            this.normalsBuffer?.Release( );
            this.quadsBuffer?.Release( );

            this.voxelsBuffer   = null;
            this.edgesBuffer    = null;
            this.cornersBuffer  = null;
            this.verticesBuffer = null;
            this.normalsBuffer  = null;
            this.quadsBuffer    = null;

            this.bufferResolution = -1;
        }

        // the next contour makes them all anew
        public void releaseBuffers( ) {
            this.releaseGridBuffers( );

            this.configurationBuffer?.Release( );
            this.densityFunctionsBuffer?.Release( );
            this.vertexCountsBuffer?.Release( );
            this.countsBuffer?.Release( );
            this.argumentsBuffer?.Release( );

            this.configurationBuffer    = null;
            this.densityFunctionsBuffer = null;
            this.vertexCountsBuffer     = null;
            this.countsBuffer           = null;
            this.argumentsBuffer        = null;
        }

    }

    public override Mesh readMesh( GPUVoxelization voxelization ) {
        return SurfaceExtractor.readMesh( voxelization );
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
