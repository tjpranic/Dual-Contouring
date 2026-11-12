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
using QEFSolverType   = QEFSolver.Type;

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

    // how many numbers the 1D and 2D points of the corners of every row along z take up between them
    // (BurstOccupancy.Points)
    [BurstCompile( CompileSynchronously = true )]
    private struct CountPointsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<int> rowCounts;

        public void Execute( int row ) {
            var corners = this.grid.resolution + 1;
            var total   = 0;

            for( var z = 0; z < corners; ++z ) {
                var corner = this.grid.cornerOf( ( row * corners ) + z );

                if( BurstOccupancy.mayHavePoints( this.grid, this.solidCorners, corner ) ) {
                    total += BurstOccupancy.offsetOf( this.grid, this.materials, corner, 3, 3 );
                }
            }

            this.rowCounts[row] = total;
        }

    }

    // section 5.1: the 1D point of every edge of the grid that the contour crosses, and where the points of every
    // corner start. The points of the corners of a row follow each other from where the row starts
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SearchEdgesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<byte>                 solidCorners;
        [ReadOnly] public NativeArray<int>                  rowStarts;

        public BurstGrid grid;
        public int       edgeSearchIterations;

        [NativeDisableParallelForRestriction] public NativeArray<int>   starts;
        [NativeDisableParallelForRestriction] public NativeArray<float> values;

        public void Execute( int row ) {
            var corners = this.grid.resolution + 1;
            var next    = this.rowStarts[row];

            for( var z = 0; z < corners; ++z ) {
                var index  = ( row * corners ) + z;
                var corner = this.grid.cornerOf( index );

                this.starts[index] = next;

                if( !BurstOccupancy.mayHavePoints( this.grid, this.solidCorners, corner ) ) {
                    continue;
                }

                var edges = 0;

                for( var axis = 0; axis < 3; ++axis ) {
                    if( BurstOccupancy.isCrossed( this.grid, this.materials, corner, axis ) ) {
                        var startIsSolid = this.materials[index] != MaterialIndex.Void;

                        this.values[next + edges] = BurstOccupancy.searchEdge( this.grid, corner, axis, startIsSolid, this.densityFunctions, this.edgeSearchIterations );

                        ++edges;
                    }
                }

                next += BurstOccupancy.offsetOf( this.grid, this.materials, corner, 3, 3 );
            }
        }

    }

    // section 5.2: the 2D point of every line the contour draws across a face of the grid, after the 1D points of the
    // corner the face has its minimum at. It reads the 1D points at the ends of each line, which are all there by now,
    // and writes only the 2D points of its own rows
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SearchFacesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<byte>                 solidCorners;
        [ReadOnly] public NativeArray<int>                  starts;

        public BurstGrid               grid;
        public BurstOccupancy.Searches searches;

        [NativeDisableParallelForRestriction] public NativeArray<float> values;

        public void Execute( int row ) {
            var corners = this.grid.resolution + 1;

            for( var z = 0; z < corners; ++z ) {
                var index  = ( row * corners ) + z;
                var corner = this.grid.cornerOf( index );

                if( !BurstOccupancy.mayHavePoints( this.grid, this.solidCorners, corner ) ) {
                    continue;
                }

                var at = this.starts[index] + BurstOccupancy.offsetOf( this.grid, this.materials, corner, 3, 0 );

                for( var axis = 0; axis < 3; ++axis ) {
                    if( !BurstOccupancy.hasFace( this.grid, corner, axis ) ) {
                        continue;
                    }

                    var faceCorners = BurstOccupancy.faceCornersOf( this.grid, this.materials, corner, axis );
                    var lines       = BurstOccupancy.linesOf( faceCorners );

                    if( lines > 0 ) {
                        BurstOccupancy.searchFaceLines( this.grid, corner, axis, faceCorners, this.searches, this.materials, this.solidCorners, this.densityFunctions, this.starts, this.values, at );

                        at += 2 * lines;
                    }
                }
            }
        }

    }

    // the vertices of the cells: one for every piece of the contour in a cell. The vertices of a cell follow each other,
    // in the order the table numbers them, and those of the cells of a row follow each other from where the row starts
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SolveCellsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;
        [ReadOnly] public NativeArray<int>           rowStarts;

        public BurstGrid             grid;
        public BurstOccupancy.Points points;
        public QEFSolverType         qefSolverType;
        public int                   minimizerIterations;
        public bool                  keepVerticesInCells;

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
                    BurstOccupancyCells.solve(
                        this.grid,
                        cell,
                        solid,
                        lookup,
                        vertex,
                        this.points,
                        this.qefSolverType,
                        this.minimizerIterations,
                        this.keepVerticesInCells,
                        this.materials,
                        out var position,
                        out var normal
                    );

                    this.positions[next + vertex] = position;
                    this.normals[next + vertex]   = normal;
                }

                next += count;
            }
        }

    }

    // what the jobs share. It is a class of its own so that Burst code does not reach into the voxelizer for it
    private static class BurstOccupancyCells {

        // section 5.3, one vertex of a cell: the minimizer of the QEF of the planes at the 1D points of its piece of the
        // contour, kept inside the cell unless that is turned off. Its normal is the sum of the normals of those planes.
        // A piece that has no plane at all has its vertex at the average of its 1D points, facing the way its edges run
        // from solid to empty
        public static unsafe void solve(
            in BurstGrid               grid,
            int3                       cell,
            int                        solid,
            int                        lookup,
            int                        vertex,
            in BurstOccupancy.Points   found,
            QEFSolverType              qefSolverType,
            int                        minimizerIterations,
            bool                       keepVerticesInCells,
            NativeArray<MaterialIndex> materials,
            out float3                 position,
            out float3                 normal
        ) {
            const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

            var edges   = stackalloc int[EdgeCount];
            var points  = stackalloc float3[EdgeCount];
            var normals = stackalloc float3[EdgeCount];

            var count = BurstOccupancy.planesOf( grid, cell, solid, lookup, vertex, found, materials, edges, points, normals );

            var qef       = BurstQEF.create( qefSolverType, minimizerIterations );
            var sum       = float3.zero;
            var direction = float3.zero;

            normal = float3.zero;

            for( var index = 0; index < count; ++index ) {
                sum += points[index];

                direction += directionOf( solid, edges[index] );

                if( math.any( normals[index] != float3.zero ) ) {
                    qef.add( points[index], normals[index] );

                    normal += normals[index];
                }
            }

            // the average of the 1D points, which is inside the cell
            var average = sum / count;

            if( qef.intersectionCount == 0 ) {
                position = average;
                normal   = BurstSurfaceExtractor.normalize( direction );

                return;
            }

            qef.solve( grid.centerOf( cell ), out var minimizer, out _ );

            position = keepVerticesInCells ? BurstOccupancy.intoCell( grid, cell, lookup, vertex, average, minimizer ) : minimizer;
            normal   = BurstSurfaceExtractor.normalize( normal );
        }

        // the edge of a cell along the given axis at its maximum, which the cell starts the polygon around: the corners
        // of the grid at its ends, whether the contour crosses it, which end is solid, and the materials there
        public struct Edge {

            public int3          start;
            public int3          end;
            public bool          crossed;
            public bool          startIsSolid;
            public MaterialIndex material;

        }

        // that edge is edge 3, 7 or 11 of the cell, which run from corner 4, 2 and 6 to corner 7
        public static Edge edgeOf( in BurstGrid grid, int3 cell, int axis, NativeArray<MaterialIndex> materials ) {
            var start = grid.cornerOfCell( cell, axis == 0 ? 4 : axis == 1 ? 2 : 6 );
            var end   = grid.cornerOfCell( cell, 7 );

            var startMaterial = materials[grid.cornerIndex( start )];
            var endMaterial   = materials[grid.cornerIndex( end )];

            return new Edge {
                start        = start,
                end          = end,
                crossed      = BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ),
                startIsSolid = startMaterial != MaterialIndex.Void,
                material     = startMaterial == MaterialIndex.Void ? endMaterial : startMaterial
            };
        }

        // the vertices that the edge four cells have in common is associated with in each of them, going around the
        // edge so that the triangle of the first three faces its empty end. BurstQuads.clusterOf gives the cells row by
        // row: going around the edge they are the first, the second, the fourth and the third, which faces the start
        // of the edge
        public static int4 aroundEdge( in BurstGrid grid, int4 cluster, int axis, bool startIsSolid, NativeArray<byte> solidCorners, NativeArray<int> vertexStarts ) {
            var vertices = int4.zero;

            for( var place = 0; place < 4; ++place ) {
                var lookup = BurstCells.vertexLookupOf( grid, solidCorners, cluster[place] );

                vertices[place] = vertexStarts[cluster[place]] + BurstTables.lookupEdgeVertex( lookup, BurstQuads.edgeOf( axis, place ) );
            }

            return startIsSolid ? vertices.xzwy : vertices.xywz;
        }

        // along an edge of a cell, from its solid end to its empty one
        public static float3 directionOf( int solid, int edge ) {
            var direction = float3.zero;

            direction[edge / 4] = ( solid & ( 1 << BurstTables.lookupEdgeCorner( edge, 0 ) ) ) != 0 ? 1.0f : -1.0f;

            return direction;
        }

    }

    // the polygons each cell starts, two bits for each axis: around the edge of the cell along that axis at its maximum,
    // where the contour crosses the edge and the four cells around it are in the grid. Each of those cells then has a
    // vertex for the edge. The bits say how the polygon is split into triangles (BurstOccupancy.splitOf)
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SplitQuadsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;
        [ReadOnly] public NativeArray<int>           vertexStarts;
        [ReadOnly] public NativeArray<float3>        positions;

        public BurstGrid             grid;
        public BurstOccupancy.Points points;
        public bool                  splitQuads;

        [WriteOnly] public NativeArray<byte> quads;

        public void Execute( int index ) {
            var solid = this.solidCorners[index];
            var quads = 0;

            if( solid != 0 && solid != 255 ) {
                var cell = this.grid.cellOf( index );

                for( var axis = 0; axis < 3; ++axis ) {
                    if( !BurstQuads.clusterOf( this.grid, cell, axis, out var cluster ) ) {
                        continue;
                    }

                    var edge = BurstOccupancyCells.edgeOf( this.grid, cell, axis, this.materials );

                    if( !edge.crossed ) {
                        continue;
                    }

                    var split = BurstOccupancy.SecondDiagonal;

                    if( this.splitQuads ) {
                        var around = BurstOccupancyCells.aroundEdge( this.grid, cluster, axis, edge.startIsSolid, this.solidCorners, this.vertexStarts );
                        var along  = this.points.alongEdge( this.grid, this.materials, edge.start, axis );

                        split = BurstOccupancy.splitOf(
                            this.positions[around.x],
                            this.positions[around.y],
                            this.positions[around.z],
                            this.positions[around.w],
                            this.grid.positionOf( edge.startIsSolid ? edge.start : edge.end ),
                            this.grid.positionOf( edge.startIsSolid ? edge.end : edge.start ),
                            BurstOccupancy.positionOnEdge( this.grid, edge.start, axis, along ),
                            BurstOccupancy.tolerance / this.grid.resolution
                        );
                    }

                    quads |= split << ( 2 * axis );
                }
            }

            this.quads[index] = ( byte )quads;
        }

    }

    // the vertices and triangles of the mesh, cell by cell and axis by axis, a vertex when a polygon first uses it. The
    // same volumes give the same mesh, vertex for vertex
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct ContourJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;
        [ReadOnly] public NativeArray<int>           vertexStarts;
        [ReadOnly] public NativeArray<float3>        positions;
        [ReadOnly] public NativeArray<float3>        normals;
        [ReadOnly] public NativeArray<byte>          quads;

        public BurstGrid             grid;
        public BurstOccupancy.Points points;

        // the index each vertex has in the mesh plus one, zero until a polygon uses it
        public NativeArray<int> meshVertices;

        public NativeList<float3>             meshPositions;
        public NativeList<float3>             meshNormals;
        public NativeList<BurstMesh.Triangle> triangles;

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
                    var split = ( quads >> ( 2 * axis ) ) & 3;

                    if( split == BurstOccupancy.NoPolygon ) {
                        continue;
                    }

                    BurstQuads.clusterOf( this.grid, cell, axis, out var cluster );

                    var edge   = BurstOccupancyCells.edgeOf( this.grid, cell, axis, this.materials );
                    var around = BurstOccupancyCells.aroundEdge( this.grid, cluster, axis, edge.startIsSolid, this.solidCorners, this.vertexStarts );

                    var subMeshIndex = BurstSurfaceExtractor.findHighestMaterialBit( edge.material );

                    var v0 = this.meshVertexOf( around.x );
                    var v1 = this.meshVertexOf( around.y );
                    var v2 = this.meshVertexOf( around.z );
                    var v3 = this.meshVertexOf( around.w );

                    // every triangle goes around the same way as the polygon, which makes it face the empty end of
                    // the edge
                    switch( split ) {
                        case BurstOccupancy.FirstDiagonal: {
                            this.triangles.Add( new BurstMesh.Triangle( v0, v1, v2, subMeshIndex ) );
                            this.triangles.Add( new BurstMesh.Triangle( v0, v2, v3, subMeshIndex ) );

                            break;
                        }
                        case BurstOccupancy.SecondDiagonal: {
                            this.triangles.Add( new BurstMesh.Triangle( v0, v1, v3, subMeshIndex ) );
                            this.triangles.Add( new BurstMesh.Triangle( v3, v1, v2, subMeshIndex ) );

                            break;
                        }
                        default: {
                            // the 1D point of the edge becomes a vertex, between the normals of the four around it
                            var along = this.points.alongEdge( this.grid, this.materials, edge.start, axis );

                            this.meshPositions.Add( BurstOccupancy.positionOnEdge( this.grid, edge.start, axis, along ) );
                            this.meshNormals.Add( BurstSurfaceExtractor.normalize( this.normals[around.x] + this.normals[around.y] + this.normals[around.z] + this.normals[around.w] ) );

                            var middle = this.meshPositions.Length - 1;

                            this.triangles.Add( new BurstMesh.Triangle( middle, v0, v1, subMeshIndex ) );
                            this.triangles.Add( new BurstMesh.Triangle( middle, v1, v2, subMeshIndex ) );
                            this.triangles.Add( new BurstMesh.Triangle( middle, v2, v3, subMeshIndex ) );
                            this.triangles.Add( new BurstMesh.Triangle( middle, v3, v0, subMeshIndex ) );

                            break;
                        }
                    }
                }
            }
        }

    }

    // the debug view: a voxel for every vertex, in its cell, in the order of the cells, and the corners and edges of
    // those cells, each the first time a cell has it. There are no densities: a corner shows -1 inside and 1 outside.
    // An edge the contour crosses shows its 1D point, and the normal of the plane there in the cell that showed the edge
    // first: each of the cells around an edge has a plane of its own at that point. Where that cell has no plane there,
    // it shows the direction from the solid end of the edge to the empty one
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct DebugInformationJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<byte>          solidCorners;
        [ReadOnly] public NativeArray<int>           vertexStarts;
        [ReadOnly] public NativeArray<float3>        positions;
        [ReadOnly] public NativeArray<float3>        normals;

        public BurstGrid             grid;
        public BurstOccupancy.Points points;

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

        // the edge of a cell, with the point and normal the cell has for it if the contour crosses it
        private int edgeOf( int3 cell, int edge, float3 intersection, float3 normal ) {
            var key = this.grid.edgeOfCell( cell, edge );

            if( this.edgeIndices.TryGetValue( key, out var index ) ) {
                return index;
            }

            index = this.edges.Length;

            this.edges.Add( new BurstDebugInformation.EdgeData {
                corner0      = this.cornerOf( this.grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) ) ),
                corner1      = this.cornerOf( this.grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 1 ) ) ),
                intersection = intersection,
                normal       = normal
            } );

            this.edgeIndices.Add( key, index );

            return index;
        }

        public unsafe void Execute( ) {
            const int EdgeCount = SurfaceExtractor.Voxel.EdgeCount;

            // the points and normals of the pieces of a cell, and those of all its pieces by edge of the cell
            var edges   = stackalloc int[EdgeCount];
            var points  = stackalloc float3[EdgeCount];
            var normals = stackalloc float3[EdgeCount];

            var edgePoints  = stackalloc float3[EdgeCount];
            var edgeNormals = stackalloc float3[EdgeCount];

            for( var index = 0; index < this.solidCorners.Length; ++index ) {
                var solid = this.solidCorners[index];

                if( solid == 0 || solid == 255 ) {
                    continue;
                }

                var cell   = this.grid.cellOf( index );
                var lookup = BurstCells.vertexLookupOf( this.grid, this.solidCorners, index );
                var count  = BurstTables.lookupVertexCount( lookup );

                for( var edge = 0; edge < EdgeCount; ++edge ) {
                    edgePoints[edge]  = float3.zero;
                    edgeNormals[edge] = float3.zero;
                }

                for( var vertex = 0; vertex < count; ++vertex ) {
                    var pieceEdges = BurstOccupancy.planesOf( this.grid, cell, solid, lookup, vertex, this.points, this.materials, edges, points, normals );

                    for( var pieceEdge = 0; pieceEdge < pieceEdges; ++pieceEdge ) {
                        var hasPlane = math.any( normals[pieceEdge] != float3.zero );

                        edgePoints[edges[pieceEdge]]  = points[pieceEdge];
                        edgeNormals[edges[pieceEdge]] = hasPlane ? normals[pieceEdge] : BurstOccupancyCells.directionOf( solid, edges[pieceEdge] );
                    }
                }

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
                    for( var edge = 0; edge < EdgeCount; ++edge ) {
                        data.setEdge( edge, this.edgeOf( cell, edge, edgePoints[edge], edgeNormals[edge] ) );
                    }

                    this.voxels.Add( data );
                }
            }
        }

    }

    private static readonly ProfilerMarker burstSampleCornersMarker = new( "OccupancyDualContouring.Burst.SampleCorners" );
    private static readonly ProfilerMarker burstSearchPointsMarker  = new( "OccupancyDualContouring.Burst.SearchPoints" );
    private static readonly ProfilerMarker burstSolveVerticesMarker = new( "OccupancyDualContouring.Burst.SolveVertices" );
    private static readonly ProfilerMarker burstContourMarker       = new( "OccupancyDualContouring.Burst.Contour" );
    private static readonly ProfilerMarker burstBuildMeshMarker     = new( "OccupancyDualContouring.Burst.BuildMesh" );
    private static readonly ProfilerMarker burstDebugMarker         = new( "OccupancyDualContouring.Burst.DebugInformation" );

    private Either<CPUVoxelization, GPUVoxelization> voxelizeBurst( IEnumerable<DensityFunction> densityFunctions ) {
        // a uniform grid of 2^resolution cells across, as uniform dual contouring has
        var grid = new BurstGrid( 1 << this.resolution );

        var searches = new BurstOccupancy.Searches {
            edgeIterations = this.edgeSearchIterations,
            faceSamples    = this.faceSearchSamples,
            faceIterations = this.faceSearchIterations
        };

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

        // the 1D and 2D points, laid out by rows of corners as the vertices are by rows of cells
        using var pointRowCounts = new NativeArray<int>( grid.cornerRowCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var pointRowStarts = new NativeArray<int>( grid.cornerRowCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var pointStarts    = new NativeArray<int>( grid.cornerCount,    Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var pointRoom      = new NativeArray<int>( 1, Allocator.TempJob );

        burstSearchPointsMarker.Begin( );

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

        var countPoints = new CountPointsJob {
            materials    = materials,
            solidCorners = solidCorners,
            grid         = grid,
            rowCounts    = pointRowCounts
        }.Schedule( grid.cornerRowCount, BurstBatches.rows, findSolidCorners );

        var layoutVertices = new BurstLayoutRowsJob {
            rowCounts = rowCounts,
            rowStarts = rowStarts,
            room      = room
        }.Schedule( countVertices );

        var layoutPoints = new BurstLayoutRowsJob {
            rowCounts = pointRowCounts,
            rowStarts = pointRowStarts,
            room      = pointRoom
        }.Schedule( countPoints );

        JobHandle.CombineDependencies( layoutVertices, layoutPoints ).Complete( );

        using var pointValues = new NativeArray<float>( pointRoom[0], Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        var searchEdges = new SearchEdgesJob {
            densityFunctions     = functions,
            materials            = materials,
            solidCorners         = solidCorners,
            rowStarts            = pointRowStarts,
            grid                 = grid,
            edgeSearchIterations = this.edgeSearchIterations,
            starts               = pointStarts,
            values               = pointValues
        }.Schedule( grid.cornerRowCount, BurstBatches.rows );

        new SearchFacesJob {
            densityFunctions = functions,
            materials        = materials,
            solidCorners     = solidCorners,
            starts           = pointStarts,
            grid             = grid,
            searches         = searches,
            values           = pointValues
        }.Schedule( grid.cornerRowCount, BurstBatches.rows, searchEdges ).Complete( );

        burstSearchPointsMarker.End( );

        var points = new BurstOccupancy.Points {
            starts = pointStarts,
            values = pointValues
        };

        // every vertex of every cell, row after row
        using var positions = new NativeArray<float3>( room[0], Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var normals   = new NativeArray<float3>( room[0], Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

        using( burstSolveVerticesMarker.Auto( ) ) {
            new SolveCellsJob {
                materials           = materials,
                solidCorners        = solidCorners,
                rowStarts           = rowStarts,
                grid                = grid,
                points              = points,
                qefSolverType       = this.qefSolverType,
                minimizerIterations = this.minimizerIterations,
                keepVerticesInCells = this.keepVerticesInCells,
                vertexStarts        = vertexStarts,
                positions           = positions,
                normals             = normals
            }.Schedule( grid.rowCount, BurstBatches.rows ).Complete( );
        }

        using var quads         = new NativeArray<byte>( grid.cellCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var meshVertices  = new NativeArray<int>( positions.Length, Allocator.TempJob );
        using var meshPositions = new NativeList<float3>( Allocator.TempJob );
        using var meshNormals   = new NativeList<float3>( Allocator.TempJob );
        using var triangles     = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );

        using( burstContourMarker.Auto( ) ) {
            var findQuads = new SplitQuadsJob {
                materials    = materials,
                solidCorners = solidCorners,
                vertexStarts = vertexStarts,
                positions    = positions,
                grid         = grid,
                points       = points,
                splitQuads   = this.splitQuads && this.keepVerticesInCells,
                quads        = quads
            }.Schedule( grid.cellCount, BurstBatches.cells );

            new ContourJob {
                materials            = materials,
                solidCorners         = solidCorners,
                vertexStarts         = vertexStarts,
                positions            = positions,
                normals              = normals,
                quads                = quads,
                grid                 = grid,
                points               = points,
                meshVertices         = meshVertices,
                meshPositions        = meshPositions,
                meshNormals          = meshNormals,
                triangles            = triangles
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
                    materials            = materials,
                    solidCorners         = solidCorners,
                    vertexStarts         = vertexStarts,
                    positions            = positions,
                    normals              = normals,
                    grid                 = grid,
                    points               = points,
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
