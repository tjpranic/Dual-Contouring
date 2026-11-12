using System;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Combination   = DensityFunction.Combination;
using MaterialIndex = SurfaceExtractor.MaterialIndex;

// the searches of Occupancy-Based Dual Contouring, which find the surface from inside and outside alone, against where
// the surface is known to be. They run in Burst jobs, as they do in the implementation
public class OccupancySearchTest {

    public enum Surface {
        Box,
        TurnedBox,
        Sphere
    }

    private static readonly Vector3 origin  = new( 0.013f, -0.021f, 0.017f );
    private static readonly Vector3 extents = new( 0.21f, 0.33f, 0.27f );
    private const           float   radius  = 0.3f;

    private static BurstTestSupport.FixedDensityFunction shapeOf( Surface surface ) {
        return surface switch {
            Surface.Box       => new( DensityFunction.Type.Cuboid,    origin, extents, Combination.Union, MaterialIndex.Material0 ),
            Surface.TurnedBox => new( DensityFunction.Type.Cuboid,    origin, Quaternion.Euler( 30.0f, -60.0f, 75.0f ), extents, Combination.Union, MaterialIndex.Material0 ),
            Surface.Sphere    => new( DensityFunction.Type.Ellipsoid, origin, Vector3.one * radius, Combination.Union, MaterialIndex.Material0 ),
            _                 => throw new Exception( "Unknown surface specified" )
        };
    }

    // every edge of the grid: whether its ends are inside, and where the search finds the contour on the edges that
    // have one end inside and one outside
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SearchEdgesJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

        public BurstGrid grid;
        public int       iterations;

        public NativeArray<bool>  startIsSolid;
        public NativeArray<bool>  endIsSolid;
        public NativeArray<float> found;
        public NativeArray<bool>  managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var edge = 0; edge < this.found.Length; ++edge ) {
                this.grid.cornersOfEdge( edge, out var start, out var end );

                this.startIsSolid[edge] = BurstOccupancy.isSolid( this.grid.positionOf( start ), this.densityFunctions );
                this.endIsSolid[edge]   = BurstOccupancy.isSolid( this.grid.positionOf( end ),   this.densityFunctions );

                if( this.startIsSolid[edge] != this.endIsSolid[edge] ) {
                    this.found[edge] = BurstOccupancy.searchEdge( this.grid, start, edge / this.grid.edgesPerAxis, this.startIsSolid[edge], this.densityFunctions, this.iterations );
                }
            }
        }

    }

    private struct Vector {

        public double x, y, z;

        public Vector( Vector3 vector ) {
            this.x = vector.x;
            this.y = vector.y;
            this.z = vector.z;
        }

        public Vector( double x, double y, double z ) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vector operator -( Vector a, Vector b ) {
            return new Vector( a.x - b.x, a.y - b.y, a.z - b.z );
        }

        public static double dot( Vector a, Vector b ) {
            return ( a.x * b.x ) + ( a.y * b.y ) + ( a.z * b.z );
        }

    }

    // how far along the way from start to end the surface is crossed, 0 at the start and 1 at the end, worked out from
    // the shape itself in double precision. One of the two points is inside the shape and the other is not
    private static double crossingOf( Surface surface, Vector start, Vector end, bool startIsSolid ) {
        var shape = shapeOf( surface );
        var frame = new DensityFunction.Frame( shape );

        var center    = new Vector( shape.origin );
        var direction = end - start;

        if( surface == Surface.Sphere ) {
            // |start + s direction - center|^2 = radius^2
            var offset = start - center;

            var a = Vector.dot( direction, direction );
            var b = 2.0 * Vector.dot( offset, direction );
            var c = Vector.dot( offset, offset ) - ( ( double )radius * radius );

            var root = Math.Sqrt( ( b * b ) - ( 4.0 * a * c ) );

            // the way out of the sphere is the larger of the two, the way in the smaller
            return startIsSolid ? ( -b + root ) / ( 2.0 * a ) : ( -b - root ) / ( 2.0 * a );
        }

        // a box is where the point lies between two planes along each of its own axes. The way from start to end is
        // between each pair of planes from one point to another, and inside the box where it is between all three
        var axes    = new[] { new Vector( frame.xAxis ), new Vector( frame.yAxis ), new Vector( frame.zAxis ) };
        var sizes   = new double[] { shape.extents.x, shape.extents.y, shape.extents.z };
        var enters  = double.NegativeInfinity;
        var leaves  = double.PositiveInfinity;

        for( var axis = 0; axis < 3; ++axis ) {
            var position = Vector.dot( start - center, axes[axis] );
            var speed    = Vector.dot( direction,      axes[axis] );

            if( speed == 0.0 ) {
                continue;
            }

            var first  = ( -sizes[axis] - position ) / speed;
            var second = (  sizes[axis] - position ) / speed;

            enters = Math.Max( enters, Math.Min( first, second ) );
            leaves = Math.Min( leaves, Math.Max( first, second ) );
        }

        return startIsSolid ? leaves : enters;
    }

    // section 5.1: after n halvings what is left of the edge is 2^-n of it long, and the 1D point is its end inside
    [Test]
    public void theContourIsFoundOnAnEdgeToWithinTheLastHalvingFromItsSolidSide( [Values] Surface surface, [Values( 1, 4, 8, 15 )] int iterations ) {
        var grid = new BurstGrid( 16 );

        using var functions    = BurstDensityFunction.snapshot( new DensityFunction[] { shapeOf( surface ) }, Allocator.TempJob );
        using var startIsSolid = new NativeArray<bool>  ( grid.edgesPerAxis * 3, Allocator.TempJob );
        using var endIsSolid   = new NativeArray<bool>  ( grid.edgesPerAxis * 3, Allocator.TempJob );
        using var found        = new NativeArray<float> ( grid.edgesPerAxis * 3, Allocator.TempJob );
        using var managed      = new NativeArray<bool>  ( 1, Allocator.TempJob );

        new SearchEdgesJob {
            densityFunctions = functions,
            grid             = grid,
            iterations       = iterations,
            startIsSolid     = startIsSolid,
            endIsSolid       = endIsSolid,
            found            = found,
            managed          = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        // what the samples are off by through rounding, in cells
        const double rounding = 1e-5;

        var step    = Math.Pow( 2.0, -iterations );
        var crossed = 0;

        for( var edge = 0; edge < found.Length; ++edge ) {
            if( startIsSolid[edge] == endIsSolid[edge] ) {
                continue;
            }

            ++crossed;

            grid.cornersOfEdge( edge, out var start, out var end );

            var crossing = crossingOf( surface, new Vector( ( Vector3 )grid.positionOf( start ) ), new Vector( ( Vector3 )grid.positionOf( end ) ), startIsSolid[edge] );

            // from the crossing toward the solid end of the edge, which is its start or its end
            var inside = startIsSolid[edge] ? crossing - found[edge] : found[edge] - crossing;

            Assert.GreaterOrEqual( inside, -rounding,       $"how far inside the point found on edge {edge} is, in cells" );
            Assert.LessOrEqual(    inside, step + rounding, $"how far inside the point found on edge {edge} is, in cells" );
        }

        Assert.Greater( crossed, 100, "edges that the contour crosses" );
    }

    // the ends of an edge as the search places them, and a point of the edge that fifteen halvings can come to
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct PlaceOnEdgesJob : IJob {

        [ReadOnly] public NativeArray<int>   edges;
        [ReadOnly] public NativeArray<float> parts;

        public BurstGrid grid;

        public NativeArray<float3> starts;
        public NativeArray<float3> ends;
        public NativeArray<float>  along;
        public NativeArray<bool>   managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var index = 0; index < this.edges.Length; ++index ) {
                this.grid.cornersOfEdge( this.edges[index], out var start, out _ );

                var axis = this.edges[index] / this.grid.edgesPerAxis;

                this.starts[index] = BurstOccupancy.positionOnEdge( this.grid, start, axis, 0.0f );
                this.ends[index]   = BurstOccupancy.positionOnEdge( this.grid, start, axis, 1.0f );
                this.along[index]  = BurstOccupancy.positionOnEdge( this.grid, start, axis, this.parts[index] )[axis];
            }
        }

    }

    // the search samples the corners of an edge exactly where the grid has them, so it agrees with the grid on which of
    // them are inside. And on the largest grid, 256^3 cells, every point that fifteen halvings can come to is a
    // position of its own: a float still tells them apart
    [Test]
    public void pointsOnAnEdgeAreWhereTheGridHasThem( [Values( 4, 16, 256 )] int resolution ) {
        const int count = 20000;

        var grid   = new BurstGrid( resolution );
        var random = new System.Random( 7 );

        using var edges   = new NativeArray<int>    ( count, Allocator.TempJob );
        using var parts   = new NativeArray<float>  ( count, Allocator.TempJob );
        using var starts  = new NativeArray<float3> ( count, Allocator.TempJob );
        using var ends    = new NativeArray<float3> ( count, Allocator.TempJob );
        using var along   = new NativeArray<float>  ( count, Allocator.TempJob );
        using var managed = new NativeArray<bool>   ( 1, Allocator.TempJob );

        var edgesToFill = edges;
        var partsToFill = parts;

        for( var index = 0; index < count; ++index ) {
            edgesToFill[index] = random.Next( grid.edgesPerAxis * 3 );

            // a multiple of 2^-15 below 1
            partsToFill[index] = random.Next( 1 << 15 ) / ( float )( 1 << 15 );
        }

        new PlaceOnEdgesJob {
            edges   = edges,
            parts   = parts,
            grid    = grid,
            starts  = starts,
            ends    = ends,
            along   = along,
            managed = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        for( var index = 0; index < count; ++index ) {
            grid.cornersOfEdge( edges[index], out var start, out var end );

            var axis = edges[index] / grid.edgesPerAxis;

            Assert.IsTrue( math.all( grid.positionOf( start ) == starts[index] ), $"the start of edge {edges[index]}" );
            Assert.IsTrue( math.all( grid.positionOf( end )   == ends[index] ),   $"the end of edge {edges[index]}" );

            // exactly the point, worked out in double precision
            var expected = ( ( start[axis] + ( double )parts[index] ) / resolution ) - 0.5;

            Assert.AreEqual( expected, ( double )along[index], $"the point {parts[index]} of the way along edge {edges[index]}" );
        }
    }

}
