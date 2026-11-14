using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using Implementation            = SurfaceExtractor.Implementation;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using ContourVertices           = SurfaceExtractor.ContourVertices;
using QEFSolverType             = QEFSolver.Type;

public partial class UniformDualContouring {

    // the vertex of every cell the contour passes through. The other cells, which is most of them, are left without one:
    // nothing is written for them, and what the arrays hold there is never read
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SolveCellsJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<byte>                 solidCorners;

        public BurstGrid     grid;
        public BurstSettings settings;

        [WriteOnly] public NativeArray<float3> vertices;
        [WriteOnly] public NativeArray<float3> normals;
        [WriteOnly] public NativeArray<bool>   hasVertex;

        public void Execute( int index ) {
            var solid = this.solidCorners[index];

            if( solid == 0 || solid == 255 ) {
                this.hasVertex[index] = false;

                return;
            }

            var solved = BurstCells.solve(
                this.grid,
                this.grid.cellOf( index ),
                solid,
                this.settings,
                this.densityFunctions,
                this.densities,
                this.materials,
                out _,
                out var vertex,
                out var normal
            );

            this.hasVertex[index] = solved;

            if( solved ) {
                this.vertices[index] = vertex;
                this.normals[index]  = normal;
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
                if( BurstQuads.clusterOf( this.grid, cell, axis, out var cluster ) &&
                    this.hasVertex[cluster.y] && this.hasVertex[cluster.z] && this.hasVertex[cluster.w] &&
                    BurstSurfaceExtractor.intersectsContour( this.materials[BurstQuads.edgeStartOf( this.grid, cell, axis )], this.materials[BurstQuads.edgeEndOf( this.grid, cell )] )
                ) {
                    quads |= 1 << axis;
                }
            }

            this.quads[index] = ( byte )quads;
        }

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
            // most cells start no quad, and eight of those in a row are passed over in one go. A grid has a whole number
            // of eights of cells unless it is a single cell
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
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
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
                vertex          = this.hasVertex[index] ? this.vertices[index] : float3.zero,
                normal          = this.hasVertex[index] ? this.normals[index]  : float3.zero,
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
            ContourVertices              contourVertices
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

            using var solidCorners = new NativeArray<byte>( grid.cellCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );

            using( sampleCornersMarker.Auto( ) ) {
                new BurstSampleCornersJob {
                    densityFunctions = functions,
                    grid             = grid,
                    densities        = densities,
                    materials        = materials
                }.Schedule( grid.cornerCount, BurstBatches.corners ).Complete( );
            }

            using( solveVerticesMarker.Auto( ) ) {
                var findSolidCorners = new BurstSolidCornersJob {
                    materials    = materials,
                    grid         = grid,
                    solidCorners = solidCorners
                }.Schedule( grid.rowCount, BurstBatches.rows );

                new SolveCellsJob {
                    densityFunctions = functions,
                    densities        = densities,
                    materials        = materials,
                    solidCorners     = solidCorners,
                    grid             = grid,
                    settings         = settings,
                    vertices         = vertices,
                    normals          = normals,
                    hasVertex        = hasVertex
                }.Schedule( grid.cellCount, BurstBatches.cells, findSolidCorners ).Complete( );
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
                }.Schedule( grid.cellCount, BurstBatches.scans );

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
