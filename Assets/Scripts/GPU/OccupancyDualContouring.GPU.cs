using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

using MaterialIndex   = SurfaceExtractor.MaterialIndex;
using VoxelType       = SurfaceExtractor.Voxel.Type;
using CPUVoxelization = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization = SurfaceExtractor.Implementation.GPU.Voxelization;
using VertexNormals   = SurfaceExtractor.VertexNormals;

public partial class OccupancyDualContouring {

    // Occupancy-Based Dual Contouring in a compute shader, OccupancyDualContouring.compute, whose kernels do what the
    // jobs of the Burst implementation do. What the shader keeps is laid out as BurstGrid lays out the corners and
    // cells of a grid
    private class GPUImplementation {

        // constant buffer data needs to be padded so every element starts on a 4 byte aligned address
        // https://docs.microsoft.com/en-us/windows/win32/direct3dhlsl/dx-graphics-hlsl-packing-rules
        [StructLayout( LayoutKind.Explicit )]
        public struct Configuration {
            [FieldOffset( 0 * 4 )] public int resolution;
            [FieldOffset( 1 * 4 )] public int minimizerIterations;
            [FieldOffset( 2 * 4 )] public int edgeSearchIterations;
            [FieldOffset( 3 * 4 )] public int faceSearchSamples;
            [FieldOffset( 4 * 4 )] public int faceSearchIterations;
            [FieldOffset( 5 * 4 )] public int splitQuads;
            [FieldOffset( 6 * 4 )] public int keepVerticesInCells;
            [FieldOffset( 7 * 4 )] public int densityFunctionCount;
        }

        private readonly ComputeShader shader;

        private readonly int sampleCornersKernel;
        private readonly int classifyCellsKernel;
        private readonly int searchEdgesKernel;
        private readonly int solveCellsKernel;
        private readonly int contourSharedKernel;
        private readonly int contourSplitKernel;

        private ComputeBuffer configurationBuffer;
        private ComputeBuffer tablesBuffer;
        private ComputeBuffer densityFunctionsBuffer;
        private ComputeBuffer totalsBuffer;
        private ComputeBuffer countsBuffer;
        private ComputeBuffer argumentsBuffer;

        // what is kept for every corner and cell of the grid, made anew when the grid changes
        private ComputeBuffer materialsBuffer;
        private ComputeBuffer cellsBuffer;
        private ComputeBuffer vertexStartsBuffer;
        private ComputeBuffer edgePointsBuffer;

        // the vertices of the cells and of fans, the vertices of the mesh when its normals are split, and its quads.
        // Each is made larger when a contour needs more room, and kept otherwise
        private ComputeBuffer verticesBuffer;
        private ComputeBuffer normalsBuffer;
        private ComputeBuffer splitVerticesBuffer;
        private ComputeBuffer splitNormalsBuffer;
        private ComputeBuffer quadsBuffer;

        // the grid the buffers of the corners and cells were made for, -1 for none
        private int bufferResolution = -1;

        // how many vertices the cells of the last contour have between them
        public int cellVertexCount { get; private set; }

        public ComputeBuffer materials    { get { return this.materialsBuffer; } }
        public ComputeBuffer cells        { get { return this.cellsBuffer; } }
        public ComputeBuffer vertexStarts { get { return this.vertexStartsBuffer; } }
        public ComputeBuffer edgePoints   { get { return this.edgePointsBuffer; } }
        public ComputeBuffer vertices     { get { return this.verticesBuffer; } }
        public ComputeBuffer normals      { get { return this.normalsBuffer; } }

        public GPUImplementation( ) {
            this.shader = Resources.Load<ComputeShader>( "Shaders/OccupancyDualContouring" );

            this.sampleCornersKernel = this.shader.FindKernel( "sampleCorners" );
            this.classifyCellsKernel = this.shader.FindKernel( "classifyCells" );
            this.searchEdgesKernel   = this.shader.FindKernel( "searchEdges" );
            this.solveCellsKernel    = this.shader.FindKernel( "solveCells" );
            this.contourSharedKernel = this.shader.FindKernel( "contourShared" );
            this.contourSplitKernel  = this.shader.FindKernel( "contourSplit" );
        }

        // the tables of DualMarchingCubes one after the other, in the order and at the places the shader has them
        private static int[] buildTables( ) {
            const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

            var tables = new List<int>( );

            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                tables.Add( DualMarchingCubes.lookupVertexEdges( solidCorners ).Length );
            }
            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                var vertices = DualMarchingCubes.lookupVertexEdges( solidCorners );

                for( var edge = 0; edge < EdgeCount; ++edge ) {
                    tables.Add( System.Array.FindIndex( vertices, ( edges ) => edges.Contains( edge ) ) );
                }
            }
            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                tables.Add( DualMarchingCubes.lookupJoinedFace( solidCorners ) );
            }
            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                for( var edge = 0; edge < EdgeCount; ++edge ) {
                    tables.Add( DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, edge, 0 ) );
                    tables.Add( DualMarchingCubes.lookupEdgeAcrossFace( solidCorners, edge, 1 ) );
                }
            }
            for( var solidCorners = 0; solidCorners < 256; ++solidCorners ) {
                for( var vertex = 0; vertex < 4; ++vertex ) {
                    for( var other = vertex + 1; other < 4; ++other ) {
                        DualMarchingCubes.lookupSeparation( solidCorners, vertex, other, out var normal, out var twiceOffset );

                        tables.Add( normal.x );
                        tables.Add( normal.y );
                        tables.Add( normal.z );
                        tables.Add( twiceOffset );
                    }
                }
            }
            for( var edge = 0; edge < EdgeCount; ++edge ) {
                tables.Add( DualMarchingCubes.lookupEdgeFace( edge, 0 ) );
                tables.Add( DualMarchingCubes.lookupEdgeFace( edge, 1 ) );
            }
            for( var face = 0; face < SurfaceExtractor.Voxel.FaceCount; ++face ) {
                tables.Add( DualMarchingCubes.lookupFaceAxis( face ) );
            }
            for( var face = 0; face < SurfaceExtractor.Voxel.FaceCount; ++face ) {
                tables.Add( DualMarchingCubes.lookupFaceSide( face ) );
            }
            for( var edge = 0; edge < EdgeCount; ++edge ) {
                tables.Add( DualMarchingCubes.lookupEdgeCorner( edge, 0 ) );
                tables.Add( DualMarchingCubes.lookupEdgeCorner( edge, 1 ) );
            }
            for( var corner = 0; corner < SurfaceExtractor.Voxel.CornerCount; ++corner ) {
                for( var axis = 0; axis < 3; ++axis ) {
                    tables.Add( DualMarchingCubes.lookupCornerOffset( corner, axis ) );
                }
            }

            return tables.ToArray( );
        }

        // a buffer with room for at least that many elements: the one there is if it is large enough, a larger one
        // otherwise. A compute buffer holds at least one element
        private static void reserve( ref ComputeBuffer buffer, int count, int stride ) {
            if( buffer != null && buffer.count >= count ) {
                return;
            }

            buffer?.Release( );

            // half as much again, so that a contour that grows a little at a time does not make a buffer every time
            buffer = new ComputeBuffer( Mathf.Max( 1, count + ( count / 2 ) ), stride );
        }

        public Either<CPUVoxelization, GPUVoxelization> voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          edgeSearchIterations,
            int                          faceSearchSamples,
            int                          faceSearchIterations,
            bool                         splitQuads,
            bool                         keepVerticesInCells,
            VertexNormals                vertexNormals
        ) {
            var densityFunctionData = densityFunctions.Select( ( densityFunction ) => new DensityFunction.Data( densityFunction ) ).ToArray( );

            var configuration = new Configuration {
                resolution           = resolution,
                minimizerIterations  = minimizerIterations,
                edgeSearchIterations = edgeSearchIterations,
                faceSearchSamples    = faceSearchSamples,
                faceSearchIterations = faceSearchIterations,
                splitQuads           = splitQuads ? 1 : 0,
                keepVerticesInCells  = keepVerticesInCells ? 1 : 0,
                densityFunctionCount = densityFunctionData.Length
            };

            if( this.configurationBuffer == null ) {
                var tables = buildTables( );

                this.configurationBuffer = new ComputeBuffer( 1, Marshal.SizeOf( typeof( Configuration ) ), ComputeBufferType.Constant );
                this.tablesBuffer        = new ComputeBuffer( tables.Length, sizeof( int ) );
                this.totalsBuffer        = new ComputeBuffer( 2, sizeof( int ) );
                this.countsBuffer        = new ComputeBuffer( 2, sizeof( int ) );
                this.argumentsBuffer     = new ComputeBuffer( 4, sizeof( int ), ComputeBufferType.IndirectArguments );

                this.tablesBuffer.SetData( tables );
            }

            if( resolution != this.bufferResolution ) {
                this.releaseGridBuffers( );

                var corners = ( resolution + 1 ) * ( resolution + 1 ) * ( resolution + 1 );
                var cells   = resolution * resolution * resolution;

                this.materialsBuffer    = new ComputeBuffer( corners,     sizeof( uint ) );
                this.cellsBuffer        = new ComputeBuffer( cells,       sizeof( uint ) );
                this.vertexStartsBuffer = new ComputeBuffer( cells,       sizeof( uint ) );
                this.edgePointsBuffer   = new ComputeBuffer( corners * 3, sizeof( float ) );

                this.bufferResolution = resolution;
            }

            // with no density functions the buffer holds one that is never read
            reserve( ref this.densityFunctionsBuffer, densityFunctionData.Length, Marshal.SizeOf( typeof( DensityFunction.Data ) ) );

            if( densityFunctionData.Length > 0 ) {
                this.densityFunctionsBuffer.SetData( densityFunctionData );
            }

            // every contour sets the constant buffer again, because all instances share the shader
            this.shader.SetConstantBuffer( Shader.PropertyToID( "Configuration" ), this.configurationBuffer, 0, Marshal.SizeOf( typeof( Configuration ) ) );

            this.configurationBuffer.SetData ( new Configuration[] { configuration } );
            this.totalsBuffer.SetData        ( new int[] { 0, 0 } );

            // a thread group is 4 x 4 x 4 cells or corners. The cells of a grid are a multiple of four across, its corners
            // one more than that, and the kernels over the corners leave out what is past the last one
            var cellGroups   = resolution / 4;
            var cornerGroups = ( resolution / 4 ) + 1;

            this.shader.SetBuffer ( this.sampleCornersKernel, "densityFunctions", this.densityFunctionsBuffer );
            this.shader.SetBuffer ( this.sampleCornersKernel, "materialsOut",     this.materialsBuffer );
            this.shader.Dispatch  ( this.sampleCornersKernel, cornerGroups, cornerGroups, cornerGroups );

            // a kernel that writes one of the grid's buffers has it under a name of its own (see the shader)
            this.shader.SetBuffer ( this.classifyCellsKernel, "tables",          this.tablesBuffer );
            this.shader.SetBuffer ( this.classifyCellsKernel, "materials",       this.materialsBuffer );
            this.shader.SetBuffer ( this.classifyCellsKernel, "cellsOut",        this.cellsBuffer );
            this.shader.SetBuffer ( this.classifyCellsKernel, "vertexStartsOut", this.vertexStartsBuffer );
            this.shader.SetBuffer ( this.classifyCellsKernel, "totals",          this.totalsBuffer );
            this.shader.Dispatch  ( this.classifyCellsKernel, cellGroups, cellGroups, cellGroups );

            // how many vertices the cells have between them and how many polygons there are, which says how much room
            // the rest needs. Reading it waits for the two kernels above
            var totals = new int[2];

            this.totalsBuffer.GetData( totals );

            this.cellVertexCount = totals[0];

            var polygons = totals[1];
            var split    = vertexNormals == VertexNormals.Split;

            // a polygon is two or four triangles, one or two quads, and can add a vertex on its edge. With split
            // normals every triangle has three vertices of its own
            reserve( ref this.verticesBuffer,      this.cellVertexCount + polygons, Marshal.SizeOf( typeof( Vector3 ) ) );
            reserve( ref this.normalsBuffer,       this.cellVertexCount + polygons, Marshal.SizeOf( typeof( Vector3 ) ) );
            reserve( ref this.splitVerticesBuffer, split ? polygons * 12 : 0,       Marshal.SizeOf( typeof( Vector3 ) ) );
            reserve( ref this.splitNormalsBuffer,  split ? polygons * 12 : 0,       Marshal.SizeOf( typeof( Vector3 ) ) );
            reserve( ref this.quadsBuffer,         polygons * 2,                    Marshal.SizeOf( typeof( UniformDualContouring.Quad.Data ) ) );

            // with shared normals the vertices of the cells are the first vertices of the mesh
            this.countsBuffer.SetData    ( new int[] { split ? 0 : this.cellVertexCount, 0 } );
            this.argumentsBuffer.SetData ( new int[] { 0, 1, 0, 0 } );

            this.shader.SetBuffer ( this.searchEdgesKernel, "densityFunctions", this.densityFunctionsBuffer );
            this.shader.SetBuffer ( this.searchEdgesKernel, "materials",        this.materialsBuffer );
            this.shader.SetBuffer ( this.searchEdgesKernel, "edgePointsOut",    this.edgePointsBuffer );
            this.shader.Dispatch  ( this.searchEdgesKernel, cornerGroups, cornerGroups, cornerGroups );

            this.shader.SetBuffer ( this.solveCellsKernel, "densityFunctions", this.densityFunctionsBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "tables",           this.tablesBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "materials",        this.materialsBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "cells",            this.cellsBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "vertexStarts",     this.vertexStartsBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "edgePoints",       this.edgePointsBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "vertices",         this.verticesBuffer );
            this.shader.SetBuffer ( this.solveCellsKernel, "normals",          this.normalsBuffer );
            this.shader.Dispatch  ( this.solveCellsKernel, cellGroups, cellGroups, cellGroups );

            var contourKernel = split ? this.contourSplitKernel : this.contourSharedKernel;

            this.shader.SetBuffer( contourKernel, "tables",       this.tablesBuffer );
            this.shader.SetBuffer( contourKernel, "materials",    this.materialsBuffer );
            this.shader.SetBuffer( contourKernel, "cells",        this.cellsBuffer );
            this.shader.SetBuffer( contourKernel, "vertexStarts", this.vertexStartsBuffer );
            this.shader.SetBuffer( contourKernel, "edgePoints",   this.edgePointsBuffer );
            this.shader.SetBuffer( contourKernel, "totals",       this.totalsBuffer );
            this.shader.SetBuffer( contourKernel, "vertices",     this.verticesBuffer );
            this.shader.SetBuffer( contourKernel, "normals",      this.normalsBuffer );
            this.shader.SetBuffer( contourKernel, "quads",        this.quadsBuffer );
            this.shader.SetBuffer( contourKernel, "counts",       this.countsBuffer );
            this.shader.SetBuffer( contourKernel, "arguments",    this.argumentsBuffer );

            if( split ) {
                this.shader.SetBuffer( contourKernel, "splitVertices", this.splitVerticesBuffer );
                this.shader.SetBuffer( contourKernel, "splitNormals",  this.splitNormalsBuffer );
            }

            this.shader.Dispatch( contourKernel, cellGroups, cellGroups, cellGroups );

            return new Either<CPUVoxelization, GPUVoxelization>.Type1( new( ) {
                vertices  = split ? this.splitVerticesBuffer : this.verticesBuffer,
                normals   = split ? this.splitNormalsBuffer  : this.normalsBuffer,
                quads     = this.quadsBuffer,
                corners   = this.materialsBuffer,
                edges     = this.edgePointsBuffer,
                voxels    = this.cellsBuffer,
                counts    = this.countsBuffer,
                arguments = this.argumentsBuffer
            } );
        }

        private void releaseGridBuffers( ) {
            this.materialsBuffer?.Release( );
            this.cellsBuffer?.Release( );
            this.vertexStartsBuffer?.Release( );
            this.edgePointsBuffer?.Release( );

            this.materialsBuffer    = null;
            this.cellsBuffer        = null;
            this.vertexStartsBuffer = null;
            this.edgePointsBuffer   = null;

            this.bufferResolution = -1;
        }

        // the next contour makes them all anew
        public void releaseBuffers( ) {
            this.releaseGridBuffers( );

            this.configurationBuffer?.Release( );
            this.tablesBuffer?.Release( );
            this.densityFunctionsBuffer?.Release( );
            this.totalsBuffer?.Release( );
            this.countsBuffer?.Release( );
            this.argumentsBuffer?.Release( );
            this.verticesBuffer?.Release( );
            this.normalsBuffer?.Release( );
            this.splitVerticesBuffer?.Release( );
            this.splitNormalsBuffer?.Release( );
            this.quadsBuffer?.Release( );

            this.configurationBuffer    = null;
            this.tablesBuffer           = null;
            this.densityFunctionsBuffer = null;
            this.totalsBuffer           = null;
            this.countsBuffer           = null;
            this.argumentsBuffer        = null;
            this.verticesBuffer         = null;
            this.normalsBuffer          = null;
            this.splitVerticesBuffer    = null;
            this.splitNormalsBuffer     = null;
            this.quadsBuffer            = null;
        }

    }

    // made the first time it is used, so the voxelizer can contour without having been started
    private GPUImplementation gpuImplementation;

    private Either<CPUVoxelization, GPUVoxelization> voxelizeGPU( IEnumerable<DensityFunction> densityFunctions ) {
        this.gpuImplementation ??= new GPUImplementation( );

        return this.gpuImplementation.voxelize(
            densityFunctions,
            1 << this.resolution,
            this.minimizerIterations,
            this.edgeSearchIterations,
            this.faceSearchSamples,
            this.faceSearchIterations,
            this.splitQuads,
            this.keepVerticesInCells,
            this.contourNormals
        );
    }

    public override void release( ) {
        base.release( );
        this.gpuImplementation?.releaseBuffers( );
    }

    public override Mesh readMesh( GPUVoxelization voxelization ) {
        return SurfaceExtractor.readMesh( voxelization );
    }

    // the mesh and the debug view of the contour the GPU implementation made last: a voxel for every vertex, in its
    // cell, in the order of the cells, and the corners and edges of those cells, as the Burst implementation shows
    // them. The shader does not keep the planes it places a vertex from, so an edge the contour crosses shows its 1D
    // point and the direction from its solid end to its empty one
    public override CPUVoxelization convert( GPUVoxelization voxelization ) {
        const int EdgeCount   = SurfaceExtractor.Voxel.EdgeCount;
        const int CornerCount = SurfaceExtractor.Voxel.CornerCount;

        var gpu  = this.gpuImplementation;
        var grid = new BurstGrid( 1 << this.resolution );

        var materials    = new int[grid.cornerCount];
        var cells        = new uint[grid.cellCount];
        var vertexStarts = new uint[grid.cellCount];
        var edgePoints   = new float[grid.cornerCount * 3];
        var positions    = new Vector3[gpu.cellVertexCount];
        var normals      = new Vector3[gpu.cellVertexCount];

        gpu.materials.GetData    ( materials );
        gpu.cells.GetData        ( cells );
        gpu.vertexStarts.GetData ( vertexStarts );
        gpu.edgePoints.GetData   ( edgePoints );

        if( gpu.cellVertexCount > 0 ) {
            gpu.vertices.GetData ( positions, 0, 0, gpu.cellVertexCount );
            gpu.normals.GetData  ( normals,   0, 0, gpu.cellVertexCount );
        }

        var corners = new List<BurstDebugInformation.CornerData>( );
        var edges   = new List<BurstDebugInformation.EdgeData>( );
        var voxels  = new List<BurstDebugInformation.VoxelData>( );

        var cornerIndices = new Dictionary<int, int>( );
        var edgeIndices   = new Dictionary<int, int>( );

        int cornerOf( int3 position ) {
            var index = grid.cornerIndex( position );

            if( !cornerIndices.TryGetValue( index, out var corner ) ) {
                corner = corners.Count;

                corners.Add( new BurstDebugInformation.CornerData {
                    position      = grid.positionOf( position ),
                    density       = materials[index] != 0 ? -1.0f : 1.0f,
                    materialIndex = ( MaterialIndex )materials[index]
                } );

                cornerIndices.Add( index, corner );
            }

            return corner;
        }

        int edgeOf( int3 cell, int edge ) {
            var key = grid.edgeOfCell( cell, edge );

            if( edgeIndices.TryGetValue( key, out var index ) ) {
                return index;
            }

            var axis  = edge / 4;
            var start = grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) );
            var end   = grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 1 ) );

            var data = new BurstDebugInformation.EdgeData {
                corner0 = cornerOf( start ),
                corner1 = cornerOf( end )
            };

            var startIsSolid = materials[grid.cornerIndex( start )] != 0;
            var endIsSolid   = materials[grid.cornerIndex( end )] != 0;

            if( startIsSolid != endIsSolid ) {
                data.intersection = BurstOccupancy.positionOnEdge( grid, start, axis, edgePoints[( grid.cornerIndex( start ) * 3 ) + axis] );

                data.normal[axis] = startIsSolid ? 1.0f : -1.0f;
            }

            index = edges.Count;

            edges.Add( data );
            edgeIndices.Add( key, index );

            return index;
        }

        for( var index = 0; index < cells.Length; ++index ) {
            var count = ( int )( cells[index] >> 16 );
            var cell  = grid.cellOf( index );

            for( var vertex = 0; vertex < count; ++vertex ) {
                var data = new BurstDebugInformation.VoxelData {
                    type            = VoxelType.None,
                    center          = grid.centerOf( cell ),
                    size            = new float3( 1.0f / grid.resolution ),
                    vertex          = positions[vertexStarts[index] + vertex],
                    normal          = normals[vertexStarts[index] + vertex],
                    hasFeaturePoint = true
                };

                for( var corner = 0; corner < CornerCount; ++corner ) {
                    data.setCorner( corner, cornerOf( grid.cornerOfCell( cell, corner ) ) );
                }
                for( var edge = 0; edge < EdgeCount; ++edge ) {
                    data.setEdge( edge, edgeOf( cell, edge ) );
                }

                voxels.Add( data );
            }
        }

        using var cornerData = new NativeArray<BurstDebugInformation.CornerData>( corners.ToArray( ), Allocator.Temp );
        using var edgeData   = new NativeArray<BurstDebugInformation.EdgeData>( edges.ToArray( ), Allocator.Temp );
        using var voxelData  = new NativeArray<BurstDebugInformation.VoxelData>( voxels.ToArray( ), Allocator.Temp );

        var ( debugCorners, debugEdges, debugVoxels ) = BurstDebugInformation.convert( cornerData, edgeData, voxelData );

        return new CPUVoxelization {
            mesh    = SurfaceExtractor.readMesh( voxelization ),
            corners = debugCorners,
            edges   = debugEdges,
            voxels  = debugVoxels
        };
    }

}
