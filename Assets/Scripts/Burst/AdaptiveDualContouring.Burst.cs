using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using CPUVoxelization           = SurfaceExtractor.Implementation.CPU.Voxelization;
using GPUVoxelization           = SurfaceExtractor.Implementation.GPU.Voxelization;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using ChildCornerMaterials      = BurstOctreeTopology.ChildCornerMaterials;

public partial class AdaptiveDualContouring {

    // the state of every node of the octree, as the managed voxels keep it. A node is a leaf in the managed octree when
    // it is a leaf or collapsed (Pseudo): the nodes below a collapsed node are still there, and nothing looks at them
    // any more
    private struct BurstNodes : IDisposable {

        public NativeArray<VoxelType> types;
        public NativeArray<bool>      hasFeaturePoint;
        public NativeArray<float3>    vertices;
        public NativeArray<float3>    normals;
        public NativeArray<BurstQEF>  qefs;
        public NativeArray<bool>      hasQEF;

        // every node starts out internal, VoxelType.Internal is zero, without a vertex or a QEF
        public BurstNodes( int count ) {
            this.types           = new NativeArray<VoxelType> ( count, Allocator.TempJob );
            this.hasFeaturePoint = new NativeArray<bool>      ( count, Allocator.TempJob );
            this.vertices        = new NativeArray<float3>    ( count, Allocator.TempJob );
            this.normals         = new NativeArray<float3>    ( count, Allocator.TempJob );
            this.qefs            = new NativeArray<BurstQEF>  ( count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
            this.hasQEF          = new NativeArray<bool>      ( count, Allocator.TempJob );
        }

        public void Dispose( ) {
            this.types.Dispose( );
            this.hasFeaturePoint.Dispose( );
            this.vertices.Dispose( );
            this.normals.Dispose( );
            this.qefs.Dispose( );
            this.hasQEF.Dispose( );
        }

    }

    // the leaves, as the managed implementation finds the vertex of every voxel. It does that for the internal nodes
    // from their own corners as well, which nothing reads afterwards: simplification gives an internal node a QEF of its
    // own from its children, and contouring and the debug view only look at the nodes that are not internal
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct SolveLeavesJob : IJobParallelFor {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;

        public BurstOctree   octree;
        public BurstSettings settings;

        [NativeDisableParallelForRestriction] public NativeArray<VoxelType> types;
        [NativeDisableParallelForRestriction] public NativeArray<bool>      hasFeaturePoint;
        [NativeDisableParallelForRestriction] public NativeArray<float3>    vertices;
        [NativeDisableParallelForRestriction] public NativeArray<float3>    normals;
        [NativeDisableParallelForRestriction] public NativeArray<BurstQEF>  qefs;
        [NativeDisableParallelForRestriction] public NativeArray<bool>      hasQEF;

        public void Execute( int local ) {
            var grid = this.octree.grid;
            var node = this.octree.levelStart( this.octree.depth ) + local;

            // a leaf the contour does not pass through has no QEF at all, one it passes through always has samples
            var solved = BurstCells.solve( grid, grid.cellOf( local ), this.settings, this.densityFunctions, this.densities, this.materials, out var qef, out var vertex, out var normal );

            this.types[node]           = VoxelType.Leaf;
            this.hasFeaturePoint[node] = solved;
            this.vertices[node]        = vertex;
            this.normals[node]         = normal;
            this.qefs[node]            = qef;
            this.hasQEF[node]          = solved;
        }

    }

    // one level of the simplification, the managed climb for the nodes of that level. Every node only looks at its own
    // children, which the level below has finished with
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct SimplifyLevelJob : IJobParallelFor {

        [ReadOnly] public NativeArray<MaterialIndex> materials;

        public BurstOctree   octree;
        public BurstSettings settings;
        public int           level;
        public float         errorThreshold;
        public bool          topologySafe;

        [NativeDisableParallelForRestriction] public NativeArray<VoxelType> types;
        [NativeDisableParallelForRestriction] public NativeArray<bool>      hasFeaturePoint;
        [NativeDisableParallelForRestriction] public NativeArray<float3>    vertices;
        [NativeDisableParallelForRestriction] public NativeArray<float3>    normals;
        [NativeDisableParallelForRestriction] public NativeArray<BurstQEF>  qefs;
        [NativeDisableParallelForRestriction] public NativeArray<bool>      hasQEF;

        private MaterialIndex materialAt( int3 corner ) {
            return this.materials[this.octree.grid.cornerIndex( corner )];
        }

        public void Execute( int local ) {
            var coordinates = this.octree.nodeOf( this.level, local );
            var node        = this.octree.nodeIndex( this.level, coordinates );

            var qef           = BurstQEF.create( this.settings.qefSolverType, this.settings.minimizerIterations );
            var averageNormal = float3.zero;
            var collapsible   = true;

            for( var child = 0; child < 8; ++child ) {
                var index = this.octree.nodeIndex( this.level + 1, this.octree.childOf( coordinates, child ) );

                if( this.types[index] == VoxelType.Internal ) {
                    collapsible = false;
                }
                else if( this.hasQEF[index] ) {
                    qef.combine( this.qefs[index] );

                    averageNormal += this.normals[index];
                }
            }

            if( !collapsible ) {
                return;
            }

            if( qef.intersectionCount == 0 ) {
                // none of the children contain any of the surface, so the node is entirely inside or outside the
                // volume. It collapses into a leaf without a vertex, otherwise it would stay internal and keep every
                // node above it from collapsing
                this.types[node]  = VoxelType.Pseudo;
                this.hasQEF[node] = false;

                return;
            }

            // corner o of the child at corner c is halfway between corners c and o of the node
            var children = new ChildCornerMaterials( );
            var span     = this.octree.span( this.level + 1 );
            var minimum  = coordinates * this.octree.span( this.level );

            for( var child = 0; child < 8; ++child ) {
                for( var corner = 0; corner < 8; ++corner ) {
                    var offset = new int3(
                        BurstTables.lookupCornerOffset( child, 0 ) + BurstTables.lookupCornerOffset( corner, 0 ),
                        BurstTables.lookupCornerOffset( child, 1 ) + BurstTables.lookupCornerOffset( corner, 1 ),
                        BurstTables.lookupCornerOffset( child, 2 ) + BurstTables.lookupCornerOffset( corner, 2 )
                    );

                    children[child, corner] = this.materialAt( minimum + ( offset * span ) );
                }
            }

            if( this.topologySafe && !BurstOctreeTopology.collapsePreservesTopology( children ) ) {
                return;
            }

            var solidCorners = 0;
            for( var corner = 0; corner < 8; ++corner ) {
                if( children[corner, corner] != MaterialIndex.Void ) {
                    solidCorners |= 1 << corner;
                }
            }

            qef.placeVertex( BurstTables.lookupVertexCount( solidCorners ), this.octree.centerOf( this.level, coordinates ), out var vertex, out var error );

            if( error > this.errorThreshold ) {
                return;
            }

            this.types[node]           = VoxelType.Pseudo;
            this.hasFeaturePoint[node] = true;
            this.vertices[node]        = vertex;
            this.normals[node]         = BurstSurfaceExtractor.normalize( averageNormal );
            this.qefs[node]            = qef;
            this.hasQEF[node]          = true;
        }

    }

    // the managed contourCell, contourFace and contourEdge, in the same order, so that vertices are created and
    // triangles come out in the order the managed implementation creates them
    [BurstCompile( CompileSynchronously = true )]
    private struct ContourJob : IJob {

        [ReadOnly] public NativeArray<MaterialIndex> materials;
        [ReadOnly] public NativeArray<VoxelType>     types;
        [ReadOnly] public NativeArray<bool>          hasFeaturePoint;
        [ReadOnly] public NativeArray<float3>        vertices;
        [ReadOnly] public NativeArray<float3>        normals;

        public BurstOctree octree;

        // the index each node's vertex has in the mesh plus one, zero until a polygon uses it
        public NativeArray<int> meshVertices;

        public NativeList<float3>             meshPositions;
        public NativeList<float3>             meshNormals;
        public NativeList<BurstMesh.Triangle> triangles;

        public void Execute( ) {
            this.contourCell( new BurstNode( 0, int3.zero ) );
        }

        private int indexOf( BurstNode node ) {
            return this.octree.nodeIndex( node.level, node.coordinates );
        }

        private bool isInternal( BurstNode node ) {
            return this.types[this.indexOf( node )] == VoxelType.Internal;
        }

        private BurstNode childOf( BurstNode node, int child ) {
            return new BurstNode( node.level + 1, this.octree.childOf( node.coordinates, child ) );
        }

        // the child of an internal node, a node that is not internal stands in for its own children
        private BurstNode below( BurstNode node, int child ) {
            return this.isInternal( node ) ? this.childOf( node, child ) : node;
        }

        private void contourCell( BurstNode node ) {
            if( !this.isInternal( node ) ) {
                return;
            }

            for( var child = 0; child < 8; ++child ) {
                this.contourCell( this.childOf( node, child ) );
            }

            for( var axis = 0; axis < 3; ++axis ) {
                for( var pair = 0; pair < 4; ++pair ) {
                    this.contourFace(
                        this.childOf( node, BurstTables.lookupFacePairWithinNode( axis, pair, 0 ) ),
                        this.childOf( node, BurstTables.lookupFacePairWithinNode( axis, pair, 1 ) ),
                        axis
                    );
                }
            }

            for( var axis = 0; axis < 3; ++axis ) {
                for( var group = 0; group < 2; ++group ) {
                    this.contourEdge(
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 0 ) ),
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 1 ) ),
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 2 ) ),
                        this.childOf( node, BurstTables.lookupEdgeNodeWithinNode( axis, group, 3 ) ),
                        axis
                    );
                }
            }
        }

        private void contourFace( BurstNode first, BurstNode second, int axis ) {
            if( !this.isInternal( first ) && !this.isInternal( second ) ) {
                return;
            }

            for( var pair = 0; pair < 4; ++pair ) {
                this.contourFace(
                    this.below( first,  BurstTables.lookupFacePairWithinFacePair( axis, pair, 0 ) ),
                    this.below( second, BurstTables.lookupFacePairWithinFacePair( axis, pair, 1 ) ),
                    axis
                );
            }

            for( var edge = 0; edge < 4; ++edge ) {
                this.contourEdge(
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 0 ),
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 1 ),
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 2 ),
                    this.aroundEdgeWithinFacePair( first, second, axis, edge, 3 ),
                    BurstTables.lookupEdgeNodeWithinFacePairAxis( axis, edge )
                );
            }
        }

        // the node at the given place around one of the edges inside the face between two nodes
        private BurstNode aroundEdgeWithinFacePair( BurstNode first, BurstNode second, int axis, int edge, int place ) {
            var node = BurstTables.lookupEdgeNodeWithinFacePairNode( axis, edge, place ) == 0 ? first : second;

            return this.below( node, BurstTables.lookupEdgeNodeWithinFacePairChild( axis, edge, place ) );
        }

        private void contourEdge( BurstNode node0, BurstNode node1, BurstNode node2, BurstNode node3, int axis ) {
            if( !this.isInternal( node0 ) && !this.isInternal( node1 ) && !this.isInternal( node2 ) && !this.isInternal( node3 ) ) {
                this.generateIndices( node0, node1, node2, node3, axis );
                return;
            }

            for( var group = 0; group < 2; ++group ) {
                this.contourEdge(
                    this.below( node0, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 0 ) ),
                    this.below( node1, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 1 ) ),
                    this.below( node2, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 2 ) ),
                    this.below( node3, BurstTables.lookupEdgeNodeWithinEdgeNodes( axis, group, 3 ) ),
                    axis
                );
            }
        }

        private int meshVertexOf( BurstNode node ) {
            var index = this.indexOf( node );

            if( this.meshVertices[index] == 0 ) {
                this.meshPositions.Add( this.vertices[index] );
                this.meshNormals.Add( this.normals[index] );

                this.meshVertices[index] = this.meshPositions.Length;
            }
            return this.meshVertices[index] - 1;
        }

        private void generateIndices( BurstNode node0, BurstNode node1, BurstNode node2, BurstNode node3, int axis ) {
            // the minimal edge is an edge of the smallest of the four nodes, the first of them where several are
            var smallest     = node0;
            var smallestSlot = 0;
            if( node1.level > smallest.level ) { smallest = node1; smallestSlot = 1; }
            if( node2.level > smallest.level ) { smallest = node2; smallestSlot = 2; }
            if( node3.level > smallest.level ) { smallest = node3; smallestSlot = 3; }

            var edge = BurstTables.lookupEdgeIndexWithinEdgeNodes( axis, smallestSlot );

            var grid          = this.octree.grid;
            var startMaterial = this.materials[grid.cornerIndex( this.octree.cornerOfNode( smallest.level, smallest.coordinates, BurstTables.lookupEdgeCorner( edge, 0 ) ) )];
            var endMaterial   = this.materials[grid.cornerIndex( this.octree.cornerOfNode( smallest.level, smallest.coordinates, BurstTables.lookupEdgeCorner( edge, 1 ) ) )];

            if( !this.hasFeaturePoint[this.indexOf( node0 )] || !this.hasFeaturePoint[this.indexOf( node1 )] ||
                !this.hasFeaturePoint[this.indexOf( node2 )] || !this.hasFeaturePoint[this.indexOf( node3 )] ||
                !BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial )
            ) {
                return;
            }

            var materialIndex = startMaterial == MaterialIndex.Void ? endMaterial : startMaterial;
            var subMeshIndex  = BurstSurfaceExtractor.findHighestMaterialBit( materialIndex );

            var v0 = this.meshVertexOf( node0 );
            var v1 = this.meshVertexOf( node1 );
            var v2 = this.meshVertexOf( node2 );
            var v3 = this.meshVertexOf( node3 );

            // ensure quad is indexed facing outward. A minimal edge in the middle of the face of a coarser node is
            // contained by only three nodes, and the half of the quad that uses the same vertex twice is left out
            if( startMaterial == MaterialIndex.Void ) {
                this.addTriangle( v0, v1, v2, subMeshIndex );
                this.addTriangle( v3, v2, v1, subMeshIndex );
            }
            else {
                this.addTriangle( v1, v2, v3, subMeshIndex );
                this.addTriangle( v2, v1, v0, subMeshIndex );
            }
        }

        private void addTriangle( int a, int b, int c, int subMeshIndex ) {
            if( a != b && b != c && a != c ) {
                this.triangles.Add( new BurstMesh.Triangle( a, b, c, subMeshIndex ) );
            }
        }

    }

    // the debug view of the managed implementation: the nodes that are not internal, in the order the managed octree
    // lists them, node before children, and the corners and edges of those nodes, each the first time a node has it.
    // Every managed voxel works out the hermite data of its own edges when the contour passes through it, so an edge
    // has the hermite data of a material boundary when the first node that has it holds both empty and solid corners
    [BurstCompile( CompileSynchronously = true, FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.Standard )]
    private struct DebugInformationJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;
        [ReadOnly] public NativeArray<float>                densities;
        [ReadOnly] public NativeArray<MaterialIndex>        materials;
        [ReadOnly] public NativeArray<VoxelType>            types;
        [ReadOnly] public NativeArray<bool>                 hasFeaturePoint;
        [ReadOnly] public NativeArray<float3>               vertices;
        [ReadOnly] public NativeArray<float3>               normals;

        public BurstOctree   octree;
        public BurstSettings settings;

        public NativeList<BurstDebugInformation.CornerData> corners;
        public NativeList<BurstDebugInformation.EdgeData>   edges;
        public NativeList<BurstDebugInformation.VoxelData>  voxels;

        public NativeHashMap<int, int>  cornerIndices;
        public NativeHashMap<long, int> edgeIndices;

        public void Execute( ) {
            this.visit( new BurstNode( 0, int3.zero ) );
        }

        private int cornerOf( int3 position ) {
            var grid  = this.octree.grid;
            var index = grid.cornerIndex( position );

            if( !this.cornerIndices.TryGetValue( index, out var corner ) ) {
                corner = this.corners.Length;

                this.corners.Add( new BurstDebugInformation.CornerData {
                    position      = grid.positionOf( position ),
                    density       = this.densities[index],
                    materialIndex = this.materials[index]
                } );

                this.cornerIndices.Add( index, corner );
            }

            return corner;
        }

        private int edgeOf( int3 start, int3 end, bool mixed ) {
            var grid       = this.octree.grid;
            var startIndex = grid.cornerIndex( start );
            var endIndex   = grid.cornerIndex( end );
            var key        = ( ( long )startIndex * grid.cornerCount ) + endIndex;

            if( this.edgeIndices.TryGetValue( key, out var edge ) ) {
                return edge;
            }

            var data = new BurstDebugInformation.EdgeData {
                corner0 = this.cornerOf( start ),
                corner1 = this.cornerOf( end )
            };

            var startMaterial = this.materials[startIndex];
            var endMaterial   = this.materials[endIndex];

            if( BurstSurfaceExtractor.intersectsContour( startMaterial, endMaterial ) ) {
                data.intersection = BurstSurfaceExtractor.approximateIntersection(
                    grid.positionOf( start ),
                    this.densities[startIndex],
                    grid.positionOf( end ),
                    this.densities[endIndex],
                    this.densityFunctions,
                    BurstSurfaceExtractor.AnyMaterial,
                    this.settings.intersectionApproximation,
                    this.settings.binarySearchIterations
                );
                data.normal = BurstSurfaceExtractor.calculateNormal( data.intersection, this.densityFunctions, BurstSurfaceExtractor.AnyMaterial );
            }
            else if( mixed && BurstSurfaceExtractor.crossesMaterialBoundary( startMaterial, endMaterial ) ) {
                BurstSurfaceExtractor.calculateMaterialBoundary(
                    grid.positionOf( start ),
                    startMaterial,
                    grid.positionOf( end ),
                    endMaterial,
                    this.densityFunctions,
                    this.settings.intersectionApproximation,
                    this.settings.binarySearchIterations,
                    out data.intersection,
                    out data.normal
                );
            }

            edge = this.edges.Length;

            this.edges.Add( data );
            this.edgeIndices.Add( key, edge );

            return edge;
        }

        private void visit( BurstNode node ) {
            var index = this.octree.nodeIndex( node.level, node.coordinates );

            if( this.types[index] == VoxelType.Internal ) {
                for( var child = 0; child < 8; ++child ) {
                    this.visit( new BurstNode( node.level + 1, this.octree.childOf( node.coordinates, child ) ) );
                }
                return;
            }

            var data = new BurstDebugInformation.VoxelData {
                type            = this.types[index],
                center          = this.octree.centerOf( node.level, node.coordinates ),
                size            = new float3( this.octree.sizeOf( node.level ) ),
                vertex          = this.vertices[index],
                normal          = this.normals[index],
                hasFeaturePoint = this.hasFeaturePoint[index]
            };

            var solid = 0;
            for( var corner = 0; corner < 8; ++corner ) {
                var position = this.octree.cornerOfNode( node.level, node.coordinates, corner );

                data.setCorner( corner, this.cornerOf( position ) );

                if( this.materials[this.octree.grid.cornerIndex( position )] != MaterialIndex.Void ) {
                    ++solid;
                }
            }

            var mixed = solid != 0 && solid != 8;

            for( var edge = 0; edge < 12; ++edge ) {
                data.setEdge(
                    edge,
                    this.edgeOf(
                        this.octree.cornerOfNode( node.level, node.coordinates, BurstTables.lookupEdgeCorner( edge, 0 ) ),
                        this.octree.cornerOfNode( node.level, node.coordinates, BurstTables.lookupEdgeCorner( edge, 1 ) ),
                        mixed
                    )
                );
            }

            this.voxels.Add( data );
        }

    }

    private static readonly ProfilerMarker burstSampleCornersMarker = new( "AdaptiveDualContouring.Burst.SampleCorners" );
    private static readonly ProfilerMarker burstSolveVerticesMarker = new( "AdaptiveDualContouring.Burst.SolveVertices" );
    private static readonly ProfilerMarker burstSimplifyMarker      = new( "AdaptiveDualContouring.Burst.Simplify" );
    private static readonly ProfilerMarker burstContourMarker       = new( "AdaptiveDualContouring.Burst.Contour" );
    private static readonly ProfilerMarker burstBuildMeshMarker     = new( "AdaptiveDualContouring.Burst.BuildMesh" );
    private static readonly ProfilerMarker burstDebugMarker         = new( "AdaptiveDualContouring.Burst.DebugInformation" );

    private Either<CPUVoxelization, GPUVoxelization> voxelizeBurst( IEnumerable<DensityFunction> densityFunctions ) {
        // the octree has a depth equal to the resolution, the leaves are at depth resolution - 1
        var octree = new BurstOctree( this.resolution - 1 );
        var grid   = octree.grid;

        var settings = new BurstSettings {
            resolution                  = grid.resolution,
            minimizerIterations         = this.minimizerIterations,
            binarySearchIterations      = this.binarySearchIterations,
            surfaceCorrectionIterations = this.surfaceCorrectionIterations,
            qefSolverType               = this.qefSolverType,
            intersectionApproximation   = this.intersectionApproximation
        };

        using var functions = BurstDensityFunction.snapshot( densityFunctions, Allocator.TempJob );
        using var densities = new NativeArray<float>         ( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var materials = new NativeArray<MaterialIndex> ( grid.cornerCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory );
        using var nodes     = new BurstNodes( octree.nodeCount );

        using( burstSampleCornersMarker.Auto( ) ) {
            new BurstSampleCornersJob {
                densityFunctions = functions,
                grid             = grid,
                densities        = densities,
                materials        = materials
            }.Schedule( grid.cornerCount, 256 ).Complete( );
        }

        using( burstSolveVerticesMarker.Auto( ) ) {
            new SolveLeavesJob {
                densityFunctions = functions,
                densities        = densities,
                materials        = materials,
                octree           = octree,
                settings         = settings,
                types            = nodes.types,
                hasFeaturePoint  = nodes.hasFeaturePoint,
                vertices         = nodes.vertices,
                normals          = nodes.normals,
                qefs             = nodes.qefs,
                hasQEF           = nodes.hasQEF
            }.Schedule( grid.cellCount, 64 ).Complete( );
        }

        if( this.simplification ) {
            using( burstSimplifyMarker.Auto( ) ) {
                var handle = default( JobHandle );

                for( var level = octree.depth - 1; level >= 0; --level ) {
                    handle = new SimplifyLevelJob {
                        materials       = materials,
                        octree          = octree,
                        settings        = settings,
                        level           = level,
                        errorThreshold  = this.errorThreshold,
                        topologySafe    = this.topologySafe,
                        types           = nodes.types,
                        hasFeaturePoint = nodes.hasFeaturePoint,
                        vertices        = nodes.vertices,
                        normals         = nodes.normals,
                        qefs            = nodes.qefs,
                        hasQEF          = nodes.hasQEF
                    }.Schedule( 1 << ( 3 * level ), 64, handle );
                }

                handle.Complete( );
            }
        }

        using var meshVertices  = new NativeArray<int>( octree.nodeCount, Allocator.TempJob );
        using var meshPositions = new NativeList<float3>( Allocator.TempJob );
        using var meshNormals   = new NativeList<float3>( Allocator.TempJob );
        using var triangles     = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );

        using( burstContourMarker.Auto( ) ) {
            new ContourJob {
                materials       = materials,
                types           = nodes.types,
                hasFeaturePoint = nodes.hasFeaturePoint,
                vertices        = nodes.vertices,
                normals         = nodes.normals,
                octree          = octree,
                meshVertices    = meshVertices,
                meshPositions   = meshPositions,
                meshNormals     = meshNormals,
                triangles       = triangles
            }.Run( );
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
                using var cornerIndices = new NativeHashMap<int, int>  ( 1024, Allocator.TempJob );
                using var edgeIndices   = new NativeHashMap<long, int> ( 1024, Allocator.TempJob );

                new DebugInformationJob {
                    densityFunctions = functions,
                    densities        = densities,
                    materials        = materials,
                    types            = nodes.types,
                    hasFeaturePoint  = nodes.hasFeaturePoint,
                    vertices         = nodes.vertices,
                    normals          = nodes.normals,
                    octree           = octree,
                    settings         = settings,
                    corners          = debugCorners,
                    edges            = debugEdges,
                    voxels           = debugVoxels,
                    cornerIndices    = cornerIndices,
                    edgeIndices      = edgeIndices
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
