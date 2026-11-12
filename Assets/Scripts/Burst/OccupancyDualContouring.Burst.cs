using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using MaterialIndex   = SurfaceExtractor.MaterialIndex;
using VoxelType       = SurfaceExtractor.Voxel.Type;
using CPUVoxelization = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization = SurfaceExtractor.Implementation.GPU.Voxelization;

public partial class OccupancyDualContouring {

    // the materials at every corner of the grid, which is all that is sampled there: a corner is inside where it has one
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SampleCornersJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<MaterialIndex> materials;

        public void Execute( int index ) {
            this.materials[index] = BurstSurfaceExtractor.calculateMaterial( this.grid.positionOf( this.grid.cornerOf( index ) ), this.densityFunctions );
        }

    }

    // how many vertices the cells of every row along z have between them. A cell has a vertex for every piece of the
    // contour that passes through it (Manifold Dual Contouring, section III), which its solid corners tell, with those
    // of the cell across its joined face if it has one (BurstCells.vertexLookupOf)
    [BurstCompile( CompileSynchronously = true )]
    private struct CountVerticesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<byte> solidCorners;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<int> rowCounts;

        public void Execute( int row ) {
            var resolution = this.grid.resolution;
            var first      = row * resolution;
            var total      = 0;

            for( var z = 0; z < resolution; ++z ) {
                total += BurstTables.lookupVertexCount( BurstCells.vertexLookupOf( this.grid, this.solidCorners, first + z ) );
            }

            this.rowCounts[row] = total;
        }

    }

    // the vertices of the cells: one for every piece of the contour in a cell, placed from the points where the contour
    // crosses the edges of that piece. The vertices of a cell follow each other, in the order the table numbers them,
    // and those of the cells of a row follow each other from where the row starts
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SolveCellsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<byte>                 solidCorners;
        [ReadOnly] public NativeArray<int>                  rowStarts;

        public BurstGrid grid;
        public int       edgeSearchIterations;

        [NativeDisableParallelForRestriction] public NativeArray<int>    vertexStarts;
        [NativeDisableParallelForRestriction] public NativeArray<float3> positions;
        [NativeDisableParallelForRestriction] public NativeArray<float3> normals;

        public void Execute( int row ) {
            var resolution = this.grid.resolution;
            var next       = this.rowStarts[row];

            for( var z = 0; z < resolution; ++z ) {
                var index = ( row * resolution ) + z;
                var solid = this.solidCorners[index];

                this.vertexStarts[index] = next;

                // the contour does not pass through the cell
                if( solid == 0 || solid == 255 ) {
                    continue;
                }

                var lookup = BurstCells.vertexLookupOf( this.grid, this.solidCorners, index );
                var count  = BurstTables.lookupVertexCount( lookup );
                var cell   = this.grid.cellOf( index );

                for( var vertex = 0; vertex < count; ++vertex ) {
                    this.solve( cell, solid, lookup, vertex, next + vertex );
                }

                next += count;
            }
        }

        // one vertex of a cell, from the edges the table gives it
        private void solve( int3 cell, int solid, int lookup, int vertex, int target ) {
            var sum       = float3.zero;
            var direction = float3.zero;
            var points    = 0;

            for( var edge = 0; edge < SurfaceExtractor.Voxel.EdgeCount; ++edge ) {
                if( BurstTables.lookupEdgeVertex( lookup, edge ) != vertex ) {
                    continue;
                }

                // an edge runs along its axis from its first corner to its second, and the edges of an axis follow
                // each other in the edge layout
                var axis         = edge / 4;
                var startCorner  = BurstTables.lookupEdgeCorner( edge, 0 );
                var start        = this.grid.cornerOfCell( cell, startCorner );
                var startIsSolid = ( solid & ( 1 << startCorner ) ) != 0;

                var along = BurstOccupancy.searchEdge( this.grid, start, axis, startIsSolid, this.densityFunctions, this.edgeSearchIterations );

                sum += BurstOccupancy.positionOnEdge( this.grid, start, axis, along );

                // from the solid end of the edge to the empty one
                direction[axis] += startIsSolid ? 1.0f : -1.0f;

                ++points;
            }

            this.positions[target] = sum / points;
            this.normals[target]   = BurstSurfaceExtractor.normalize( direction );
        }

    }

    // the polygons each cell starts, a bit for each axis: around the edge of the cell along that axis at its maximum,
    // where the contour crosses the edge and the four cells around it are in the grid. Each of those cells then has a
    // vertex for the edge
    [BurstCompile( CompileSynchronously = true )]
    private struct FindQuadsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<byte> quads;

        public void Execute( int index ) {
            var solid = this.solidCorners[index];
            var quads = 0;

            if( solid != 0 && solid != 255 ) {
                var cell = this.grid.cellOf( index );

                for( var axis = 0; axis < 3; ++axis ) {
                    if( BurstQuads.clusterOf( this.grid, cell, axis, out _ ) &&
                        BurstSurfaceExtractor.intersectsContour( this.materials[BurstQuads.edgeStartOf( this.grid, cell, axis )], this.materials[BurstQuads.edgeEndOf( this.grid, cell )] )
                    ) {
                        quads |= 1 << axis;
                    }
                }
            }

            this.quads[index] = ( byte )quads;
        }

    }

    // the vertices and triangles of the mesh, cell by cell and axis by axis, a vertex when a polygon first uses it. The
    // same volumes give the same mesh, vertex for vertex
    [BurstCompile( CompileSynchronously = true )]
    private struct ContourJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;
        [ReadOnly] public NativeArray<int>           vertexStarts;
        [ReadOnly] public NativeArray<float3>        positions;
        [ReadOnly] public NativeArray<float3>        normals;
        [ReadOnly] public NativeArray<byte>          quads;

        public BurstGrid grid;

        // the index each vertex has in the mesh plus one, zero until a polygon uses it
        public NativeArray<int> meshVertices;

        public NativeList<float3>             meshPositions;
        public NativeList<float3>             meshNormals;
        public NativeList<BurstMesh.Triangle> triangles;

        // the vertex of a cell that the edge four cells have in common is associated with, the cell being the one at
        // the given place around that edge
        private int vertexOf( int cell, int axis, int place ) {
            var lookup = BurstCells.vertexLookupOf( this.grid, this.solidCorners, cell );

            return this.vertexStarts[cell] + BurstTables.lookupEdgeVertex( lookup, BurstQuads.edgeOf( axis, place ) );
        }

        private int meshVertexOf( int vertex ) {
            if( this.meshVertices[vertex] == 0 ) {
                this.meshPositions.Add( this.positions[vertex] );
                this.meshNormals.Add( this.normals[vertex] );

                this.meshVertices[vertex] = this.meshPositions.Length;
            }
            return this.meshVertices[vertex] - 1;
        }

        public void Execute( ) {
            // most cells start no polygon, and eight of those in a row are passed over in one go. A grid has a whole
            // number of eights of cells unless it is a single cell
            var eights = this.quads.Length % 8 == 0 ? this.quads.Reinterpret<ulong>( UnsafeUtility.SizeOf<byte>( ) ) : default;

            for( var index = 0; index < this.quads.Length; ++index ) {
                if( eights.IsCreated && ( index & 7 ) == 0 && eights[index >> 3] == 0 ) {
                    index += 7;

                    continue;
                }

                var quads = this.quads[index];

                if( quads == 0 ) {
                    continue;
                }

                var cell = this.grid.cellOf( index );

                for( var axis = 0; axis < 3; ++axis ) {
                    if( ( quads & ( 1 << axis ) ) == 0 ) {
                        continue;
                    }

                    BurstQuads.clusterOf( this.grid, cell, axis, out var cluster );

                    var startMaterial = this.materials[BurstQuads.edgeStartOf( this.grid, cell, axis )];
                    var endMaterial   = this.materials[BurstQuads.edgeEndOf( this.grid, cell )];
                    var materialIndex = startMaterial == MaterialIndex.Void ? endMaterial : startMaterial;
                    var subMeshIndex  = BurstSurfaceExtractor.findHighestMaterialBit( materialIndex );

                    var v0 = this.meshVertexOf( this.vertexOf( cluster.x, axis, 0 ) );
                    var v1 = this.meshVertexOf( this.vertexOf( cluster.y, axis, 1 ) );
                    var v2 = this.meshVertexOf( this.vertexOf( cluster.z, axis, 2 ) );
                    var v3 = this.meshVertexOf( this.vertexOf( cluster.w, axis, 3 ) );

                    // ensure quad is indexed facing outward
                    if( startMaterial == MaterialIndex.Void ) {
                        this.triangles.Add( new BurstMesh.Triangle( v0, v1, v2, subMeshIndex ) );
                        this.triangles.Add( new BurstMesh.Triangle( v3, v2, v1, subMeshIndex ) );
                    }
                    else {
                        this.triangles.Add( new BurstMesh.Triangle( v1, v2, v3, subMeshIndex ) );
                        this.triangles.Add( new BurstMesh.Triangle( v2, v1, v0, subMeshIndex ) );
                    }
                }
            }
        }

    }

    // the debug view: a voxel for every vertex, in its cell, in the order of the cells, and the corners and edges of
    // those cells, each the first time a cell has it. There are no densities: a corner shows -1 inside and 1 outside.
    // An edge the contour crosses shows the point where it does, and the direction from its solid end to its empty one
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct DebugInformationJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<byte>                 solidCorners;
        [ReadOnly] public NativeArray<int>                  vertexStarts;
        [ReadOnly] public NativeArray<float3>               positions;
        [ReadOnly] public NativeArray<float3>               normals;

        public BurstGrid grid;
        public int       edgeSearchIterations;

        public NativeList<BurstDebugInformation.CornerData> corners;
        public NativeList<BurstDebugInformation.EdgeData>   edges;
        public NativeList<BurstDebugInformation.VoxelData>  voxels;

        public NativeHashMap<int, int> cornerIndices;
        public NativeHashMap<int, int> edgeIndices;

        private int cornerOf( int3 position ) {
            var index = this.grid.cornerIndex( position );

            if( !this.cornerIndices.TryGetValue( index, out var corner ) ) {
                corner = this.corners.Length;

                this.corners.Add( new BurstDebugInformation.CornerData {
                    position      = this.grid.positionOf( position ),
                    density       = this.materials[index] != MaterialIndex.Void ? -1.0f : 1.0f,
                    materialIndex = this.materials[index]
                } );

                this.cornerIndices.Add( index, corner );
            }

            return corner;
        }

        private int edgeOf( int3 cell, int edge ) {
            var key = this.grid.edgeOfCell( cell, edge );

            if( this.edgeIndices.TryGetValue( key, out var index ) ) {
                return index;
            }

            var axis  = edge / 4;
            var start = this.grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) );
            var end   = this.grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 1 ) );

            var data = new BurstDebugInformation.EdgeData {
                corner0 = this.cornerOf( start ),
                corner1 = this.cornerOf( end )
            };

            var startMaterial = this.materials[this.grid.cornerIndex( start )];
            var endMaterial   = this.materials[this.grid.cornerIndex( end )];

            if( BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                var startIsSolid = startMaterial != MaterialIndex.Void;

                var along = BurstOccupancy.searchEdge( this.grid, start, axis, startIsSolid, this.densityFunctions, this.edgeSearchIterations );

                data.intersection = BurstOccupancy.positionOnEdge( this.grid, start, axis, along );

                data.normal[axis] = startIsSolid ? 1.0f : -1.0f;
            }

            index = this.edges.Length;

            this.edges.Add( data );
            this.edgeIndices.Add( key, index );

            return index;
        }

        public void Execute( ) {
            for( var index = 0; index < this.solidCorners.Length; ++index ) {
                var solid = this.solidCorners[index];

                if( solid == 0 || solid == 255 ) {
                    continue;
                }

                var cell  = this.grid.cellOf( index );
                var count = BurstTables.lookupVertexCount( BurstCells.vertexLookupOf( this.grid, this.solidCorners, index ) );

                for( var vertex = 0; vertex < count; ++vertex ) {
                    var data = new BurstDebugInformation.VoxelData {
                        type            = VoxelType.None,
                        center          = this.grid.centerOf( cell ),
                        size            = new float3( 1.0f / this.grid.resolution ),
                        vertex          = this.positions[this.vertexStarts[index] + vertex],
                        normal          = this.normals[this.vertexStarts[index] + vertex],
                        hasFeaturePoint = true
                    };

                    for( var corner = 0; corner < SurfaceExtractor.Voxel.CornerCount; ++corner ) {
                        data.setCorner( corner, this.cornerOf( this.grid.cornerOfCell( cell, corner ) ) );
                    }
                    for( var edge = 0; edge < SurfaceExtractor.Voxel.EdgeCount; ++edge ) {
                        data.setEdge( edge, this.edgeOf( cell, edge ) );
                    }

                    this.voxels.Add( data );
                }
            }
        }

    }

    private static readonly ProfilerMarker burstSampleCornersMarker = new( "OccupancyDualContouring.Burst.SampleCorners" );
    private static readonly ProfilerMarker burstSolveVerticesMarker = new( "OccupancyDualContouring.Burst.SolveVertices" );
    private static readonly ProfilerMarker burstContourMarker       = new( "OccupancyDualContouring.Burst.Contour" );
    private static readonly ProfilerMarker burstBuildMeshMarker     = new( "OccupancyDualContouring.Burst.BuildMesh" );
    private static readonly ProfilerMarker burstDebugMarker         = new( "OccupancyDualContouring.Burst.DebugInformation" );

    private Either<CPUVoxelization, GPUVoxelization> voxelizeBurst( IEnumerable<DensityFunction> densityFunctions ) {
        // a uniform grid of 2^resolution cells across, as uniform dual contouring has
        var grid = new BurstGrid( 1 << this.resolution );

        using var functions = BurstDensityFunction.snapshot( densityFunctions, Allocator.TempJob );
        using var materials = new NativeArray<MaterialIndex>( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        using var solidCorners = new NativeArray<byte> ( grid.cellCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var vertexStarts = new NativeArray<int>  ( grid.cellCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var room         = new NativeArray<int>  ( 1, Allocator.TempJob );

        using( burstSampleCornersMarker.Auto( ) ) {
            new SampleCornersJob {
                densityFunctions = functions,
                grid             = grid,
                materials        = materials
            }.Schedule( grid.cornerCount, BurstBatches.corners ).Complete( );
        }

        using var rowCounts = new NativeArray<int>( grid.rowCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var rowStarts = new NativeArray<int>( grid.rowCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        burstSolveVerticesMarker.Begin( );

        var findSolidCorners = new BurstSolidCornersJob {
            materials    = materials,
            grid         = grid,
            solidCorners = solidCorners
        }.Schedule( grid.rowCount, BurstBatches.rows );

        var countVertices = new CountVerticesJob {
            solidCorners = solidCorners,
            grid         = grid,
            rowCounts    = rowCounts
        }.Schedule( grid.rowCount, BurstBatches.rows, findSolidCorners );

        new BurstLayoutRowsJob {
            rowCounts = rowCounts,
            rowStarts = rowStarts,
            room      = room
        }.Schedule( countVertices ).Complete( );

        // every vertex of every cell, row after row
        using var positions = new NativeArray<float3>( room[0], Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var normals   = new NativeArray<float3>( room[0], Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        new SolveCellsJob {
            densityFunctions     = functions,
            solidCorners         = solidCorners,
            rowStarts            = rowStarts,
            grid                 = grid,
            edgeSearchIterations = this.edgeSearchIterations,
            vertexStarts         = vertexStarts,
            positions            = positions,
            normals              = normals
        }.Schedule( grid.rowCount, BurstBatches.rows ).Complete( );

        burstSolveVerticesMarker.End( );

        using var quads         = new NativeArray<byte>( grid.cellCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var meshVertices  = new NativeArray<int>( positions.Length, Allocator.TempJob );
        using var meshPositions = new NativeList<float3>( Allocator.TempJob );
        using var meshNormals   = new NativeList<float3>( Allocator.TempJob );
        using var triangles     = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );

        using( burstContourMarker.Auto( ) ) {
            var findQuads = new FindQuadsJob {
                materials    = materials,
                solidCorners = solidCorners,
                grid         = grid,
                quads        = quads
            }.Schedule( grid.cellCount, BurstBatches.scans );

            new ContourJob {
                materials     = materials,
                solidCorners  = solidCorners,
                vertexStarts  = vertexStarts,
                positions     = positions,
                normals       = normals,
                quads         = quads,
                grid          = grid,
                meshVertices  = meshVertices,
                meshPositions = meshPositions,
                meshNormals   = meshNormals,
                triangles     = triangles
            }.Schedule( findQuads ).Complete( );
        }

        UnityEngine.Mesh mesh;

        using( burstBuildMeshMarker.Auto( ) ) {
            mesh = BurstMesh.createMesh( meshPositions.AsArray( ), meshNormals.AsArray( ), triangles.AsArray( ) );
        }

        var voxelization = new CPUVoxelization {
            mesh    = mesh,
            corners = Array.Empty<SurfaceExtractor.Corner>( ),
            edges   = Array.Empty<SurfaceExtractor.Edge>( ),
            voxels  = Array.Empty<SurfaceExtractor.Voxel>( )
        };

        if( this.debugInformationWanted ) {
            using( burstDebugMarker.Auto( ) ) {
                using var debugCorners  = new NativeList<BurstDebugInformation.CornerData> ( Allocator.TempJob );
                using var debugEdges    = new NativeList<BurstDebugInformation.EdgeData>   ( Allocator.TempJob );
                using var debugVoxels   = new NativeList<BurstDebugInformation.VoxelData>  ( Allocator.TempJob );
                using var cornerIndices = new NativeHashMap<int, int>( 1024, Allocator.TempJob );
                using var edgeIndices   = new NativeHashMap<int, int>( 1024, Allocator.TempJob );

                new DebugInformationJob {
                    densityFunctions     = functions,
                    materials            = materials,
                    solidCorners         = solidCorners,
                    vertexStarts         = vertexStarts,
                    positions            = positions,
                    normals              = normals,
                    grid                 = grid,
                    edgeSearchIterations = this.edgeSearchIterations,
                    corners              = debugCorners,
                    edges                = debugEdges,
                    voxels               = debugVoxels,
                    cornerIndices        = cornerIndices,
                    edgeIndices          = edgeIndices
                }.Run( );

                var ( corners, edges, voxels ) = BurstDebugInformation.convert( debugCorners.AsArray( ), debugEdges.AsArray( ), debugVoxels.AsArray( ) );

                voxelization.corners = corners;
                voxelization.edges   = edges;
                voxelization.voxels  = voxels;
            }
        }

        return new Either<CPUVoxelization, GPUVoxelization>.Type0( voxelization );
    }

}
