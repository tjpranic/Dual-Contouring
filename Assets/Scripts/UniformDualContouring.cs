using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEngine;

// TODO: implement GPU uniform dual contouring

using MaterialIndex             = SurfaceExtractor.MaterialIndex;
using VoxelType                 = SurfaceExtractor.Voxel.Type;
using Implementation            = SurfaceExtractor.Implementation;
using IntersectionApproximation = SurfaceExtractor.IntersectionApproximation;
using QEFSolverType             = QEFSolver.Type;

public class UniformDualContouring : Voxelizer {

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

        public bool Equals( SurfaceExtractor.Corner other ) {
            return this.position.Equals( other.position );
        }

        public override int GetHashCode( ) {
            return this.position.GetHashCode( );
        }

    }

    public class Edge : SurfaceExtractor.Edge {

        public struct Data {

            public Corner.Data corner0;
            public Corner.Data corner1;
            public Vector3     intersection;
            public Vector3     normal;

            public Data( SurfaceExtractor.Edge edge ) {
                this.corner0      = new Corner.Data( edge.corners[0] );
                this.corner1      = new Corner.Data( edge.corners[2] );
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

        public bool intersectsContour( ) {
            return (
                this.corners[0].materialIndex == MaterialIndex.Void && this.corners[1].materialIndex >= MaterialIndex.Material1
            ) || (
                this.corners[1].materialIndex == MaterialIndex.Void && this.corners[0].materialIndex >= MaterialIndex.Material1
            );
        }

        public bool Equals( SurfaceExtractor.Edge other ) {
            return this.corners[0].Equals( other.corners[0] ) && this.corners[1].Equals( other.corners[1] );
        }

        public override int GetHashCode( ) {
            return this.corners[0].GetHashCode( ) ^ this.corners[1].GetHashCode( );
        }

    }

    public class Voxel : SurfaceExtractor.Voxel {

        public struct Data {

            public Vector3     center;
            public Vector3     size;
            public Vector3     extents;
            public Vector3     minimum;
            public Vector3     maximum;
            public SVDQEF.Data qef;
            public Vector3     vertex;
            public Vector3     normal;
            public int         index;

            public Corner.Data corner0;
            public Corner.Data corner1;
            public Corner.Data corner2;
            public Corner.Data corner3;
            public Corner.Data corner4;
            public Corner.Data corner5;
            public Corner.Data corner6;
            public Corner.Data corner7;

            public Edge.Data edge0;
            public Edge.Data edge1;
            public Edge.Data edge2;
            public Edge.Data edge3;
            public Edge.Data edge4;
            public Edge.Data edge5;
            public Edge.Data edge6;
            public Edge.Data edge7;
            public Edge.Data edge8;
            public Edge.Data edge9;
            public Edge.Data edge10;
            public Edge.Data edge11;

            public Data( Voxel voxel ) {
                if( voxel.qef is SVDQEF solver ) {
                    this.center  = voxel.center;
                    this.size    = voxel.size;
                    this.extents = voxel.extents;
                    this.minimum = voxel.minimum;
                    this.maximum = voxel.maximum;
                    this.qef     = new SVDQEF.Data( solver );
                    this.vertex  = voxel.vertex;
                    this.normal  = voxel.normal;
                    this.index   = voxel.index;

                    this.corner0 = new Corner.Data( voxel.corners[0] );
                    this.corner1 = new Corner.Data( voxel.corners[1] );
                    this.corner2 = new Corner.Data( voxel.corners[2] );
                    this.corner3 = new Corner.Data( voxel.corners[3] );
                    this.corner4 = new Corner.Data( voxel.corners[4] );
                    this.corner5 = new Corner.Data( voxel.corners[5] );
                    this.corner6 = new Corner.Data( voxel.corners[6] );
                    this.corner7 = new Corner.Data( voxel.corners[7] );

                    this.edge0  = new Edge.Data( voxel.edges[0] );
                    this.edge1  = new Edge.Data( voxel.edges[1] );
                    this.edge2  = new Edge.Data( voxel.edges[2] );
                    this.edge3  = new Edge.Data( voxel.edges[3] );
                    this.edge4  = new Edge.Data( voxel.edges[4] );
                    this.edge5  = new Edge.Data( voxel.edges[5] );
                    this.edge6  = new Edge.Data( voxel.edges[6] );
                    this.edge7  = new Edge.Data( voxel.edges[7] );
                    this.edge8  = new Edge.Data( voxel.edges[8] );
                    this.edge9  = new Edge.Data( voxel.edges[9] );
                    this.edge10 = new Edge.Data( voxel.edges[10] );
                    this.edge11 = new Edge.Data( voxel.edges[11] );
                }
                throw new Exception( "Cannot create voxel data, incompatible QEF type" );
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

        public QEFSolver qef    { get; set; }
        public Vector3   vertex { get; set; } = Vector3.zero;
        public Vector3   normal { get; set; } = Vector3.zero;
        public int       index  { get; set; } = -1;

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

        public bool hasFeaturePoint( ) {
            return this.index > -1;
        }

        public bool Equals( SurfaceExtractor.Voxel other ) {
            return this.center == other.center && this.size == other.size;
        }

        public override int GetHashCode( ) {
            return this.center.GetHashCode( ) ^ this.size.GetHashCode( );
        }

    }

    private class CPUImplementation {

        private Voxel[,,] grid;

        public (
            Mesh                                 mesh,
            IEnumerable<SurfaceExtractor.Corner> corners,
            IEnumerable<SurfaceExtractor.Edge>   edges,
            IEnumerable<SurfaceExtractor.Voxel>  voxels
        ) voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            QEFSolverType                qefSolver,
            IntersectionApproximation    intersectionApproximationMode
        ) {

            // create uniformly subdivided voxel grid

            this.grid = new Voxel[resolution, resolution, resolution];
            for( var x = 0; x < this.grid.GetLength( 0 ); ++x ) {
                for( var y = 0; y < this.grid.GetLength( 1 ); ++y ) {
                    for( var z = 0; z < this.grid.GetLength( 2 ); ++z ) {
                        var size   = Vector3.one / resolution;                 // center voxel relative to 0, 0, 0
                        var center = ( new Vector3( x, y, z ) / resolution ) - ( size / 2 * ( resolution - 1 ) );

                        this.grid[x, y, z] = new Voxel( center, size );
                    }
                }
            }

            // sample corner densities

            foreach( var voxel in this.grid ) {
                foreach( var corner in voxel.corners ) {
                    foreach( var densityFunction in densityFunctions ) {
                        ( corner.density, corner.materialIndex ) = SurfaceExtractor.calculateDensityAndMaterial( corner, densityFunction );
                    }
                }
            }

            // find contour intersections and calculate minimizing vertices

            var index = 0;
            foreach( var voxel in this.grid ) {
                if(
                    voxel.corners.All( ( corner ) => corner.materialIndex == SurfaceExtractor.MaterialIndex.Void      ) ||
                    voxel.corners.All( ( corner ) => corner.materialIndex >= SurfaceExtractor.MaterialIndex.Material1 )
                ) {
                    // cell is either fully inside or outside the volume, skip
                    continue;
                }

                voxel.qef = qefSolver switch {
                    QEFSolverType.Simple => new SimpleQEF ( minimizerIterations, surfaceCorrectionIterations ),
                    QEFSolverType.SVD    => new SVDQEF    ( minimizerIterations, surfaceCorrectionIterations ),
                    _                    => throw new Exception( "Unknown solver type specified" )
                };

                foreach( var edge in voxel.edges ) {
                    if( !edge.intersectsContour( ) ) {
                        continue;
                    }

                    edge.intersection = SurfaceExtractor.approximateIntersection ( edge, densityFunctions, intersectionApproximationMode, binarySearchIterations );
                    edge.normal       = SurfaceExtractor.calculateNormal         ( edge, densityFunctions );

                    voxel.qef.add( edge.intersection, edge.normal );
                }

                ( voxel.vertex, voxel.normal, _ ) = voxel.qef.solve( voxel, densityFunctions );

                voxel.index = index++;
            }

            // generate vertices and indices

            var vertices = new List<Vector3>( );
            var normals  = new List<Vector3>( );
            var indices  = new Dictionary<int, List<int>>( );

            for( var x = 0; x < this.grid.GetLength( 0 ); ++x ) {
                for( var y = 0; y < this.grid.GetLength( 1 ); ++y ) {
                    for( var z = 0; z < this.grid.GetLength( 2 ); ++z ) {
                        var voxel = this.grid[x, y, z];

                        if( !voxel.hasFeaturePoint( ) ) {
                            continue;
                        }

                        vertices.Add ( voxel.vertex );
                        normals.Add  ( voxel.normal );

                        // on every positive axis, generate indices using 4 voxel surrounding a common edge

                        // x axis
                        if( y + 1 < this.grid.GetLength( 1 ) && z + 1 < this.grid.GetLength( 2 ) ) {
                            var voxels = new Voxel[] {
                                voxel,
                                this.grid[x, y,     z + 1],
                                this.grid[x, y + 1, z    ],
                                this.grid[x, y + 1, z + 1]
                            };
                            var edge = voxel.edges[3]; // common edge surrounded by all 4 voxels, refer to edge layout diagram

                            this.generateIndices( voxels, edge, indices );
                        }

                        // y axis
                        if( x + 1 < this.grid.GetLength( 0 ) && z + 1 < this.grid.GetLength( 2 ) ) {
                            var voxels = new Voxel[] {
                                voxel,
                                this.grid[x + 1, y, z    ],
                                this.grid[x,     y, z + 1],
                                this.grid[x + 1, y, z + 1],
                            };
                            var edge = voxel.edges[7];

                            this.generateIndices( voxels, edge, indices );
                        }

                        // z axis
                        if( x + 1 < this.grid.GetLength( 0 ) && y + 1 < this.grid.GetLength( 1 ) ) {
                            var voxels = new Voxel[] {
                                voxel,
                                this.grid[x,     y + 1, z],
                                this.grid[x + 1, y,     z],
                                this.grid[x + 1, y + 1, z]
                            };
                            var edge = voxel.edges[11];

                            this.generateIndices( voxels, edge, indices );
                        }
                    }
                }
            }

            // build mesh
            var mesh = new Mesh {
                vertices = vertices.ToArray( ),
                normals  = normals.ToArray( )
            };

            mesh.subMeshCount = indices.Keys.Count;

            var subMeshCount = 0;
            foreach( var triangles in indices ) {
                mesh.SetTriangles( triangles.Value, subMeshCount );
                ++subMeshCount;
            }

            // build debug information
            var debugVoxels = this.grid.flatten( );

            var debugEdges = debugVoxels.Aggregate(
                new List<SurfaceExtractor.Edge>( ),
                ( accumulator, voxel ) => {
                    accumulator.AddRange( voxel.edges );
                    return accumulator;
                }
            ).Distinct( );

            var debugCorners = debugVoxels.Aggregate(
                new List<SurfaceExtractor.Corner>( ),
                ( accumulator, voxel ) => {
                    accumulator.AddRange( voxel.corners );
                    return accumulator;
                }
            ).Distinct( );

            return ( mesh, debugCorners, debugEdges, debugVoxels );
        }

        private void generateIndices( Voxel[] voxels, SurfaceExtractor.Edge edge, Dictionary<int, List<int>> indices ) {
            if( voxels.All( ( voxel ) => voxel.hasFeaturePoint( ) ) && edge.intersectsContour( ) ) {

                // indices should only be generated from void - solid intersections
                UnityEngine.Debug.Assert(
                    ( edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void && edge.corners[1].materialIndex >= SurfaceExtractor.MaterialIndex.Material1 ) ||
                    ( edge.corners[1].materialIndex == SurfaceExtractor.MaterialIndex.Void && edge.corners[0].materialIndex >= SurfaceExtractor.MaterialIndex.Material1 )
                );

                int[] triangles;

                // ensure quad is indexed facing outward
                if( edge.corners[0].materialIndex == SurfaceExtractor.MaterialIndex.Void ) {
                    triangles = new int[] {
                        voxels[0].index,
                        voxels[1].index,
                        voxels[2].index,
                        voxels[3].index,
                        voxels[2].index,
                        voxels[1].index
                    };
                }
                else {
                    triangles = new int[] {
                        voxels[1].index,
                        voxels[2].index,
                        voxels[3].index,
                        voxels[2].index,
                        voxels[1].index,
                        voxels[0].index
                    };
                }

                var subMeshIndex = SurfaceExtractor.findHighestMaterialBit( edge );
                if( !indices.ContainsKey( subMeshIndex ) ) {
                    indices.Add( subMeshIndex, new( ) );
                }
                indices[subMeshIndex].AddRange( triangles );
            }
        }

    }

    public class GPUImplementation {

        // constant buffer data needs to be padded so every element starts on a 4 byte aligned address
        // https://docs.microsoft.com/en-us/windows/win32/direct3dhlsl/dx-graphics-hlsl-packing-rules
        [StructLayout( LayoutKind.Explicit )]
        public struct Configuration {
            [FieldOffset( 0 * 4 )] public int resolution;
            [FieldOffset( 1 * 4 )] public int minimizerIterations;
            [FieldOffset( 2 * 4 )] public int binarySearchIterations;
            [FieldOffset( 3 * 4 )] public int surfaceCorrectionIterations;
            [FieldOffset( 4 * 4 )] public int intersectionApproximationMode;

            public Configuration(
                int                       resolution,
                int                       minimizerIterations,
                int                       binarySearchIterations,
                int                       surfaceCorrectionIterations,
                IntersectionApproximation intersectionApproximationMode
            ) {
                this.resolution                    = resolution;
                this.minimizerIterations           = minimizerIterations;
                this.binarySearchIterations        = binarySearchIterations;
                this.surfaceCorrectionIterations   = surfaceCorrectionIterations;
                this.intersectionApproximationMode = ( int )intersectionApproximationMode;
            }
        }

        private readonly ComputeShader uniformDualContouring;

        private ComputeBuffer configurationBuffer;
        private ComputeBuffer voxelsBuffer;
        //...

        private readonly int buildVoxelGridKernel;
        //...

        public GPUImplementation( ) {
            this.uniformDualContouring = Resources.Load<ComputeShader>( "Shaders/UniformDualContouring" );

            this.buildVoxelGridKernel = this.uniformDualContouring.FindKernel( "buildVoxelGrid" );
            //...
        }

        public (
            Mesh                                 mesh,
            IEnumerable<SurfaceExtractor.Corner> corners,
            IEnumerable<SurfaceExtractor.Edge>   edges,
            IEnumerable<SurfaceExtractor.Voxel>  voxels
        ) voxelize(
            IEnumerable<DensityFunction> densityFunctions,
            int                          resolution,
            int                          minimizerIterations,
            int                          binarySearchIterations,
            int                          surfaceCorrectionIterations,
            IntersectionApproximation    intersectionApproximation
        ) {
            var configuration = new Configuration( resolution, minimizerIterations, binarySearchIterations, surfaceCorrectionIterations, intersectionApproximation );

            this.createBuffers( configuration );

            this.buildVoxelGrid( resolution );
            //...

            this.releaseBuffers( );

            return ( null, null, null, null );
        }

        private void buildVoxelGrid( int resolution ) {
            this.uniformDualContouring.SetBuffer ( this.buildVoxelGridKernel, "voxels", this.voxelsBuffer );
            this.uniformDualContouring.Dispatch  ( this.buildVoxelGridKernel, resolution, resolution, resolution );
        }

        //...

        private void createBuffers( Configuration configuration ) {
            this.releaseBuffers( );

            var voxels = new Voxel.Data[configuration.resolution * configuration.resolution * configuration.resolution];
            //...

            this.configurationBuffer = new ComputeBuffer( 1,             Marshal.SizeOf( typeof( Configuration ) ), ComputeBufferType.Constant );
            this.voxelsBuffer        = new ComputeBuffer( voxels.Length, Marshal.SizeOf( typeof( Voxel.Data ) ) );
            //...

            var configurationConstantBuffer = Shader.PropertyToID( "Configuration" );
            this.uniformDualContouring.SetConstantBuffer( configurationConstantBuffer, this.configurationBuffer, 0, Marshal.SizeOf( typeof( Configuration ) ) );

            this.configurationBuffer.SetData ( new Configuration[] { configuration } );
            this.voxelsBuffer.SetData        ( voxels );
            //...
        }

        private void releaseBuffers( ) {
            this.configurationBuffer?.Release( );
            this.voxelsBuffer?.Release( );
        }

    }

    [Space( )]

    [SerializeField( )]
    private Implementation _implementation = Implementation.CPU;
    public override Implementation implementation {
        get { return this._implementation;  }
        set { this._implementation = value; }
    }

    [SerializeField( )]
    private IntersectionApproximation _intersectionApproximation = IntersectionApproximation.BinarySearch;
    public override IntersectionApproximation intersectionApproximation {
        get { return this._intersectionApproximation;  }
        set { this._intersectionApproximation = value; }
    }

    [SerializeField( )]
    private QEFSolverType _qefSolver = QEFSolverType.Simple;
    public override QEFSolverType qefSolver {
        get { return this._qefSolver;  }
        set { this._qefSolver = value; }
    }

    private Mesh _mesh;
    public override Mesh mesh {
        get { return this._mesh; }
    }

    private IEnumerable<SurfaceExtractor.Corner> _corners;
    public override IEnumerable<SurfaceExtractor.Corner> corners {
        get { return this._corners; }
    }

    private IEnumerable<SurfaceExtractor.Edge> _edges;
    public override IEnumerable<SurfaceExtractor.Edge> edges {
        get { return this._edges; }
    }

    private IEnumerable<SurfaceExtractor.Voxel> _voxels;
    public override IEnumerable<SurfaceExtractor.Voxel> voxels {
        get { return this._voxels; }
    }

    private CPUImplementation cpuImplementation;
    private GPUImplementation gpuImplementation;

    public override void Start( ) {
        this.cpuImplementation = new CPUImplementation( );
        this.gpuImplementation = new GPUImplementation( );

        base.Start( );
    }

    public override void voxelize( IEnumerable<DensityFunction> densityFunctions ) {
        if( this.implementation == Implementation.GPU ) {
            ( this._mesh, this._corners, this._edges, this._voxels ) = this.gpuImplementation.voxelize(
                densityFunctions,
                this.resolution,
                this.minimizerIterations,
                this.binarySearchIterations,
                this.surfaceCorrectionIterations,
                this.intersectionApproximation
            );
        }
        else {
            ( this._mesh, this._corners, this._edges, this._voxels ) = this.cpuImplementation.voxelize(
                densityFunctions,
                this.resolution,
                this.minimizerIterations,
                this.binarySearchIterations,
                this.surfaceCorrectionIterations,
                this.qefSolver,
                this.intersectionApproximation
            );
        }
    }

}

[CustomEditor( typeof( UniformDualContouring ) )]
public class UniformDualContouringEditor : VoxelizerEditor {

    public override void OnInspectorGUI( ) {
        base.OnInspectorGUI( );
    }

}