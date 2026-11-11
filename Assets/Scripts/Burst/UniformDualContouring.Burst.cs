using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using Implementation            = SurfaceExtractor.Implementation;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using VertexNormals             = SurfaceExtractor.VertexNormals;
using QEFSolverType             = QEFSolver.Type;

public partial class UniformDualContouring {

    // where the corners, cells and edges of a grid of n^3 cells are in the arrays of the Burst implementation. Corner (i,
    // j, k) is a point of the grid of (n + 1)^3, cell (x, y, z) the cell whose minimum is corner (x, y, z). Cells are in
    // the order the managed implementation goes through them, x outermost. An edge is numbered within its axis by the
    // corner it starts at, and the edges of the three axes follow each other
    private readonly struct BurstGrid {

        public readonly int resolution;

        public BurstGrid( int resolution ) {
            this.resolution = resolution;
        }

        public int cornerCount {
            get { return ( this.resolution + 1 ) * ( this.resolution + 1 ) * ( this.resolution + 1 ); }
        }

        public int cellCount {
            get { return this.resolution * this.resolution * this.resolution; }
        }

        // edges along each axis: n of them along the axis times n + 1 across each of the other two
        public int edgesPerAxis {
            get { return this.resolution * ( this.resolution + 1 ) * ( this.resolution + 1 ); }
        }

        public int cornerIndex( int3 corner ) {
            return ( ( ( corner.x * ( this.resolution + 1 ) ) + corner.y ) * ( this.resolution + 1 ) ) + corner.z;
        }

        public int3 cornerOf( int index ) {
            var points = this.resolution + 1;
            return new int3( index / ( points * points ), ( index / points ) % points, index % points );
        }

        public int cellIndex( int3 cell ) {
            return ( ( ( cell.x * this.resolution ) + cell.y ) * this.resolution ) + cell.z;
        }

        public int3 cellOf( int index ) {
            return new int3( index / ( this.resolution * this.resolution ), ( index / this.resolution ) % this.resolution, index % this.resolution );
        }

        // the corners of the grid are exactly where the corners of the managed voxels are: n is a power of two
        public float3 positionOf( int3 corner ) {
            return ( new float3( corner ) / this.resolution ) - 0.5f;
        }

        public float3 centerOf( int3 cell ) {
            return ( ( new float3( cell ) + 0.5f ) / this.resolution ) - 0.5f;
        }

        // the corner of the grid that the given corner of a cell is
        public int3 cornerOfCell( int3 cell, int corner ) {
            return cell + new int3( BurstTables.lookupCornerOffset( corner, 0 ), BurstTables.lookupCornerOffset( corner, 1 ), BurstTables.lookupCornerOffset( corner, 2 ) );
        }

        // the edge of the grid that the given edge of a cell is
        public int edgeOfCell( int3 cell, int edge ) {
            var axis  = edge / 4;
            var start = this.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) );
            var size  = new int3( this.resolution + 1 );

            size[axis] = this.resolution;

            return ( axis * this.edgesPerAxis ) + ( ( ( ( start.x * size.y ) + start.y ) * size.z ) + start.z );
        }

        public void cornersOfEdge( int edge, out int3 start, out int3 end ) {
            var axis  = edge / this.edgesPerAxis;
            var index = edge % this.edgesPerAxis;
            var size  = new int3( this.resolution + 1 );

            size[axis] = this.resolution;

            start = new int3( index / ( size.y * size.z ), ( index / size.z ) % size.y, index % size.z );
            end   = start;

            end[axis] += 1;
        }

    }

    // the settings of a contour, for the jobs to read
    private struct BurstSettings {

        public int                       resolution;
        public int                       minimizerIterations;
        public int                       binarySearchIterations;
        public int                       surfaceCorrectionIterations;
        public QEFSolverType             qefSolverType;
        public IntersectionApproximation intersectionApproximation;

    }

    // the density and material at every corner of the grid
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct SampleCornersJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<float>         densities;
        [WriteOnly] public NativeArray<MaterialIndex> materials;

        public void Execute( int index ) {
            var position = this.grid.positionOf( this.grid.cornerOf( index ) );

            this.densities[index] = BurstSurfaceExtractor.calculateDensityAndMaterial( position, this.densityFunctions, out var materialIndex );
            this.materials[index] = materialIndex;
        }

    }

    // the vertex of every cell the contour passes through, as the managed implementation finds it: the hermite data of
    // the edges of the cell in the order of the edge layout, the vertex placed from the QEF of those samples, and the
    // average of the normals of the contour samples. Each cell works out the hermite data of its own edges, so an edge
    // that the contour crosses is worked out by each of the four cells around it, as it is in the managed implementation
    // too. That keeps the data of the edges, almost all of which the contour does not cross, out of memory
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct SolveCellsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;

        public BurstGrid     grid;
        public BurstSettings settings;

        [WriteOnly] public NativeArray<float3> vertices;
        [WriteOnly] public NativeArray<float3> normals;
        [WriteOnly] public NativeArray<bool>   hasVertex;

        public void Execute( int index ) {
            var cell = this.grid.cellOf( index );

            var solidCorners = 0;
            for( var corner = 0; corner < 8; ++corner ) {
                if( this.materials[this.grid.cornerIndex( this.grid.cornerOfCell( cell, corner ) )] != MaterialIndex.Void ) {
                    solidCorners |= 1 << corner;
                }
            }

            this.vertices[index]  = float3.zero;
            this.normals[index]   = float3.zero;
            this.hasVertex[index] = false;

            // cell is either fully inside or outside the volume, skip
            if( solidCorners == 0 || solidCorners == 255 ) {
                return;
            }

            var qef           = BurstQEF.create( this.settings.qefSolverType, this.settings.minimizerIterations );
            var averageNormal = float3.zero;

            for( var edge = 0; edge < 12; ++edge ) {
                var start = this.grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 0 ) );
                var end   = this.grid.cornerOfCell( cell, BurstTables.lookupEdgeCorner( edge, 1 ) );

                var startIndex = this.grid.cornerIndex( start );
                var endIndex   = this.grid.cornerIndex( end );

                var startMaterial = this.materials[startIndex];
                var endMaterial   = this.materials[endIndex];

                if( BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                    var intersection = BurstSurfaceExtractor.approximateIntersection(
                        this.grid.positionOf( start ),
                        this.densities[startIndex],
                        this.grid.positionOf( end ),
                        this.densities[endIndex],
                        this.densityFunctions,
                        BurstSurfaceExtractor.AnyMaterial,
                        this.settings.intersectionApproximation,
                        this.settings.binarySearchIterations
                    );
                    var normal = BurstSurfaceExtractor.calculateNormal( intersection, this.densityFunctions, BurstSurfaceExtractor.AnyMaterial );

                    qef.add( intersection, normal );

                    averageNormal += normal;
                }
                else if( BurstSurfaceExtractor.crossesMaterialBoundary( startMaterial, endMaterial ) ) {
                    // the boundary between two materials draws the vertex to where they meet on the contour. Its
                    // normal is not one of the contour, and stays out of the normal of the vertex
                    BurstSurfaceExtractor.calculateMaterialBoundary(
                        this.grid.positionOf( start ),
                        startMaterial,
                        this.grid.positionOf( end ),
                        endMaterial,
                        this.densityFunctions,
                        this.settings.intersectionApproximation,
                        this.settings.binarySearchIterations,
                        out var intersection,
                        out var normal
                    );

                    qef.add( intersection, normal );
                }
            }

            if( qef.intersectionCount > 0 ) {
                qef.placeVertex( BurstTables.lookupVertexCount( solidCorners ), this.grid.centerOf( cell ), out var vertex, out _ );

                var vertexNormal = BurstSurfaceExtractor.normalize( averageNormal );

                this.vertices[index]  = BurstSurfaceExtractor.surfaceCorrection( vertex, vertexNormal, this.densityFunctions, this.settings.surfaceCorrectionIterations );
                this.normals[index]   = vertexNormal;
                this.hasVertex[index] = true;
            }
        }

    }

    // the quads each cell starts, a bit for each positive axis: around the edge of the cell along that axis at its
    // maximum, where the four cells around the edge have a vertex and the edge crosses the contour
    [BurstCompile( CompileSynchronously = true )]
    private struct FindQuadsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<bool>          hasVertex;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<byte> quads;

        public void Execute( int index ) {
            this.quads[index] = 0;

            if( !this.hasVertex[index] ) {
                return;
            }

            var cell  = this.grid.cellOf( index );
            var quads = 0;

            for( var axis = 0; axis < 3; ++axis ) {
                if( clusterOf( this.grid, cell, axis, out var cluster ) &&
                    this.hasVertex[cluster.y] && this.hasVertex[cluster.z] && this.hasVertex[cluster.w] &&
                    BurstSurfaceExtractor.intersectsContour( this.materials[edgeStartOf( this.grid, cell, axis )], this.materials[edgeEndOf( this.grid, cell )] )
                ) {
                    quads |= 1 << axis;
                }
            }

            this.quads[index] = ( byte )quads;
        }

    }

    // the four cells around the edge of a cell along the given axis at its maximum, in the order the managed
    // implementation takes them, if they are all in the grid
    private static bool clusterOf( BurstGrid grid, int3 cell, int axis, out int4 cluster ) {
        var n = grid.resolution;

        cluster = int4.zero;

        switch( axis ) {
            case 0:
                if( cell.y + 1 >= n || cell.z + 1 >= n ) {
                    return false;
                }
                cluster = new int4( grid.cellIndex( cell ), grid.cellIndex( cell + new int3( 0, 0, 1 ) ), grid.cellIndex( cell + new int3( 0, 1, 0 ) ), grid.cellIndex( cell + new int3( 0, 1, 1 ) ) );
                return true;
            case 1:
                if( cell.x + 1 >= n || cell.z + 1 >= n ) {
                    return false;
                }
                cluster = new int4( grid.cellIndex( cell ), grid.cellIndex( cell + new int3( 1, 0, 0 ) ), grid.cellIndex( cell + new int3( 0, 0, 1 ) ), grid.cellIndex( cell + new int3( 1, 0, 1 ) ) );
                return true;
            default:
                if( cell.x + 1 >= n || cell.y + 1 >= n ) {
                    return false;
                }
                cluster = new int4( grid.cellIndex( cell ), grid.cellIndex( cell + new int3( 0, 1, 0 ) ), grid.cellIndex( cell + new int3( 1, 0, 0 ) ), grid.cellIndex( cell + new int3( 1, 1, 0 ) ) );
                return true;
        }
    }

    // that edge is edge 3, 7 or 11 of the cell, which run from corner 4, 2 and 6 to corner 7
    private static int edgeStartOf( BurstGrid grid, int3 cell, int axis ) {
        var corner = axis == 0 ? 4 : axis == 1 ? 2 : 6;

        return grid.cornerIndex( grid.cornerOfCell( cell, corner ) );
    }

    private static int edgeEndOf( BurstGrid grid, int3 cell ) {
        return grid.cornerIndex( grid.cornerOfCell( cell, 7 ) );
    }

    // the vertices and triangles of the mesh, in the order the managed implementation creates them: cell by cell and axis
    // by axis, a vertex when a quad first uses it. That makes the mesh the managed one, vertex for vertex
    [BurstCompile( CompileSynchronously = true )]
    private struct ContourJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<float3>        vertices;
        [ReadOnly] public NativeArray<float3>        normals;
        [ReadOnly] public NativeArray<byte>          quads;

        public BurstGrid grid;

        // the index each cell's vertex has in the mesh plus one, zero until a quad uses it
        public NativeArray<int> meshVertices;

        public NativeList<float3>             meshPositions;
        public NativeList<float3>             meshNormals;
        public NativeList<BurstMesh.Triangle> triangles;

        private int meshVertexOf( int cell ) {
            if( this.meshVertices[cell] == 0 ) {
                this.meshPositions.Add( this.vertices[cell] );
                this.meshNormals.Add( this.normals[cell] );

                this.meshVertices[cell] = this.meshPositions.Length;
            }
            return this.meshVertices[cell] - 1;
        }

        public void Execute( ) {
            for( var index = 0; index < this.quads.Length; ++index ) {
                var quads = this.quads[index];

                if( quads == 0 ) {
                    continue;
                }

                var cell = this.grid.cellOf( index );

                for( var axis = 0; axis < 3; ++axis ) {
                    if( ( quads & ( 1 << axis ) ) == 0 ) {
                        continue;
                    }

                    clusterOf( this.grid, cell, axis, out var cluster );

                    var startMaterial = this.materials[edgeStartOf( this.grid, cell, axis )];
                    var endMaterial   = this.materials[edgeEndOf( this.grid, cell )];
                    var materialIndex = startMaterial == MaterialIndex.Void ? endMaterial : startMaterial;
                    var subMeshIndex  = BurstSurfaceExtractor.findHighestMaterialBit( materialIndex );

                    var v0 = this.meshVertexOf( cluster.x );
                    var v1 = this.meshVertexOf( cluster.y );
                    var v2 = this.meshVertexOf( cluster.z );
                    var v3 = this.meshVertexOf( cluster.w );

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

    // the hermite data of every edge of the grid for the debug view, as the managed implementation keeps it on its edges:
    // an intersection and normal on an edge that crosses the contour, or that crosses a material boundary in a cell the
    // contour passes through, and zero on the others. The managed implementation only works out the edges of those
    // cells, which leaves material boundaries deep inside a solid without hermite data
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct DebugEdgesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;

        public BurstGrid     grid;
        public BurstSettings settings;

        [WriteOnly] public NativeArray<BurstDebugInformation.EdgeData> edges;

        private bool isMixed( int3 cell ) {
            var solid = 0;
            for( var corner = 0; corner < 8; ++corner ) {
                if( this.materials[this.grid.cornerIndex( this.grid.cornerOfCell( cell, corner ) )] != MaterialIndex.Void ) {
                    ++solid;
                }
            }
            return solid != 0 && solid != 8;
        }

        // whether one of the up to four cells around the edge holds both empty and solid corners
        private bool isInMixedCell( int3 start, int axis ) {
            var across = ( axis + 1 ) % 3;
            var other  = ( axis + 2 ) % 3;

            for( var acrossOffset = -1; acrossOffset <= 0; ++acrossOffset ) {
                for( var otherOffset = -1; otherOffset <= 0; ++otherOffset ) {
                    var cell = start;

                    cell[across] += acrossOffset;
                    cell[other]  += otherOffset;

                    if( math.all( cell >= 0 ) && math.all( cell < this.grid.resolution ) && this.isMixed( cell ) ) {
                        return true;
                    }
                }
            }

            return false;
        }

        public void Execute( int index ) {
            this.grid.cornersOfEdge( index, out var start, out var end );

            var startIndex = this.grid.cornerIndex( start );
            var endIndex   = this.grid.cornerIndex( end );

            var data = new BurstDebugInformation.EdgeData {
                corner0 = startIndex,
                corner1 = endIndex
            };

            var startMaterial = this.materials[startIndex];
            var endMaterial   = this.materials[endIndex];

            if( BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                data.intersection = BurstSurfaceExtractor.approximateIntersection(
                    this.grid.positionOf( start ),
                    this.densities[startIndex],
                    this.grid.positionOf( end ),
                    this.densities[endIndex],
                    this.densityFunctions,
                    BurstSurfaceExtractor.AnyMaterial,
                    this.settings.intersectionApproximation,
                    this.settings.binarySearchIterations
                );
                data.normal = BurstSurfaceExtractor.calculateNormal( data.intersection, this.densityFunctions, BurstSurfaceExtractor.AnyMaterial );
            }
            else if( BurstSurfaceExtractor.crossesMaterialBoundary( startMaterial, endMaterial ) && this.isInMixedCell( start, index / this.grid.edgesPerAxis ) ) {
                BurstSurfaceExtractor.calculateMaterialBoundary(
                    this.grid.positionOf( start ),
                    startMaterial,
                    this.grid.positionOf( end ),
                    endMaterial,
                    this.densityFunctions,
                    this.settings.intersectionApproximation,
                    this.settings.binarySearchIterations,
                    out data.intersection,
                    out data.normal
                );
            }

            this.edges[index] = data;
        }

    }

    [BurstCompile( CompileSynchronously = true )]
    private struct DebugCornersJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float>         densities;
        [ReadOnly] public NativeArray<MaterialIndex> materials;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<BurstDebugInformation.CornerData> corners;

        public void Execute( int index ) {
            this.corners[index] = new BurstDebugInformation.CornerData {
                position      = this.grid.positionOf( this.grid.cornerOf( index ) ),
                density       = this.densities[index],
                materialIndex = this.materials[index]
            };
        }

    }

    // every cell, as the managed implementation shows them: with no type of its own
    [BurstCompile( CompileSynchronously = true )]
    private struct DebugVoxelsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<float3> vertices;
        [ReadOnly] public NativeArray<float3> normals;
        [ReadOnly] public NativeArray<bool>   hasVertex;

        public BurstGrid grid;

        [WriteOnly] public NativeArray<BurstDebugInformation.VoxelData> voxels;

        public void Execute( int index ) {
            var cell = this.grid.cellOf( index );

            var data = new BurstDebugInformation.VoxelData {
                type            = VoxelType.None,
                center          = this.grid.centerOf( cell ),
                size            = new float3( 1.0f / this.grid.resolution ),
                vertex          = this.vertices[index],
                normal          = this.normals[index],
                hasFeaturePoint = this.hasVertex[index]
            };

            for( var corner = 0; corner < 8; ++corner ) {
                data.setCorner( corner, this.grid.cornerIndex( this.grid.cornerOfCell( cell, corner ) ) );
            }
            for( var edge = 0; edge < 12; ++edge ) {
                data.setEdge( edge, this.grid.edgeOfCell( cell, edge ) );
            }

            this.voxels[index] = data;
        }

    }

    private class BurstImplementation : Implementation {

        private static readonly ProfilerMarker sampleCornersMarker = new( "UniformDualContouring.Burst.SampleCorners" );
        private static readonly ProfilerMarker solveVerticesMarker = new( "UniformDualContouring.Burst.SolveVertices" );
        private static readonly ProfilerMarker contourMarker       = new( "UniformDualContouring.Burst.Contour" );
        private static readonly ProfilerMarker buildMeshMarker     = new( "UniformDualContouring.Burst.BuildMesh" );
        private static readonly ProfilerMarker debugMarker         = new( "UniformDualContouring.Burst.DebugInformation" );

        // whether to copy the corners, edges and voxels of the next contour out for the debug view or a test
        public bool debugInformation { get; set; } = false;

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
            var grid = new BurstGrid( resolution );

            var settings = new BurstSettings {
                resolution                  = resolution,
                minimizerIterations         = minimizerIterations,
                binarySearchIterations      = binarySearchIterations,
                surfaceCorrectionIterations = surfaceCorrectionIterations,
                qefSolverType               = qefSolverType,
                intersectionApproximation   = intersectionApproximation
            };

            using var functions = BurstDensityFunction.snapshot( densityFunctions, Allocator.TempJob );

            using var densities = new NativeArray<float>         ( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
            using var materials = new NativeArray<MaterialIndex> ( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
            using var vertices  = new NativeArray<float3>        ( grid.cellCount,   Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
            using var normals   = new NativeArray<float3>        ( grid.cellCount,   Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
            using var hasVertex = new NativeArray<bool>          ( grid.cellCount,   Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

            using( sampleCornersMarker.Auto( ) ) {
                new SampleCornersJob {
                    densityFunctions = functions,
                    grid             = grid,
                    densities        = densities,
                    materials        = materials
                }.Schedule( grid.cornerCount, 256 ).Complete( );
            }

            using( solveVerticesMarker.Auto( ) ) {
                new SolveCellsJob {
                    densityFunctions = functions,
                    densities        = densities,
                    materials        = materials,
                    grid             = grid,
                    settings         = settings,
                    vertices         = vertices,
                    normals          = normals,
                    hasVertex        = hasVertex
                }.Schedule( grid.cellCount, 64 ).Complete( );
            }

            using var quads         = new NativeArray<byte>( grid.cellCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
            using var meshVertices  = new NativeArray<int>( grid.cellCount, Allocator.TempJob );
            using var meshPositions = new NativeList<float3>( Allocator.TempJob );
            using var meshNormals   = new NativeList<float3>( Allocator.TempJob );
            using var triangles     = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );

            using( contourMarker.Auto( ) ) {
                var findQuads = new FindQuadsJob {
                    materials = materials,
                    hasVertex = hasVertex,
                    grid      = grid,
                    quads     = quads
                }.Schedule( grid.cellCount, 1024 );

                new ContourJob {
                    materials     = materials,
                    vertices      = vertices,
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

            using( buildMeshMarker.Auto( ) ) {
                mesh = BurstMesh.createMesh( meshPositions.AsArray( ), meshNormals.AsArray( ), triangles.AsArray( ) );
            }

            var voxelization = new CPUVoxelization {
                mesh    = mesh,
                corners = Array.Empty<SurfaceExtractor.Corner>( ),
                edges   = Array.Empty<SurfaceExtractor.Edge>( ),
                voxels  = Array.Empty<SurfaceExtractor.Voxel>( )
            };

            if( this.debugInformation ) {
                using( debugMarker.Auto( ) ) {
                    using var debugCorners = new NativeArray<BurstDebugInformation.CornerData> ( grid.cornerCount,      Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
                    using var debugEdges   = new NativeArray<BurstDebugInformation.EdgeData>   ( grid.edgesPerAxis * 3, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
                    using var debugVoxels  = new NativeArray<BurstDebugInformation.VoxelData>  ( grid.cellCount,        Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

                    var cornersJob = new DebugCornersJob {
                        densities = densities,
                        materials = materials,
                        grid      = grid,
                        corners   = debugCorners
                    }.Schedule( grid.cornerCount, 256 );

                    var edgesJob = new DebugEdgesJob {
                        densityFunctions = functions,
                        densities        = densities,
                        materials        = materials,
                        grid             = grid,
                        settings         = settings,
                        edges            = debugEdges
                    }.Schedule( grid.edgesPerAxis * 3, 64 );

                    var voxelsJob = new DebugVoxelsJob {
                        vertices  = vertices,
                        normals   = normals,
                        hasVertex = hasVertex,
                        grid      = grid,
                        voxels    = debugVoxels
                    }.Schedule( grid.cellCount, 256 );

                    JobHandle.CombineDependencies( cornersJob, edgesJob, voxelsJob ).Complete( );

                    var ( corners, edges, voxels ) = BurstDebugInformation.convert( debugCorners, debugEdges, debugVoxels );

                    voxelization.corners = corners;
                    voxelization.edges   = edges;
                    voxelization.voxels  = voxels;
                }
            }

            return new Either<CPUVoxelization, GPUVoxelization>.Type0( voxelization );
        }

    }

}
