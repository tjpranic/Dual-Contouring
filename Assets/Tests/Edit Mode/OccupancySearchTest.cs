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

    // a search along a ray in the face of a grid of 16^3 cells that has its minimum at the middle of the grid and faces
    // along z, so a point of the face is that many cells along x and y from the origin
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SearchLineJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

        public float2 start;
        public float2 direction;
        public float  range;
        public int    samples;
        public int    iterations;

        public NativeArray<float> found;
        public NativeArray<bool>  startIsSolid;
        public NativeArray<bool>  managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            var grid   = new BurstGrid( 16 );
            var corner = new int3( 8, 8, 8 );

            this.startIsSolid[0] = BurstOccupancy.isSolid( BurstOccupancy.positionOnFace( grid, corner, 2, this.start ), this.densityFunctions );

            this.found[0] = BurstOccupancy.searchLine( grid, corner, 2, this.start, this.direction, this.startIsSolid[0], this.range, this.samples, this.iterations, this.densityFunctions );
        }

    }

    // a wall from 0.25 to 0.55 cells along x from the middle of the grid, far wider than the face along y and z
    private static readonly BurstTestSupport.FixedDensityFunction wall = new(
        DensityFunction.Type.Cuboid, new Vector3( 0.4f / 16, 0.0f, 0.0f ), new Vector3( 0.15f / 16, 0.4f, 0.4f ), Combination.Union, MaterialIndex.Material0
    );

    private static float searchLine( float2 start, float2 direction, out bool startIsSolid ) {
        using var functions = BurstDensityFunction.snapshot( new DensityFunction[] { wall }, Allocator.TempJob );
        using var found     = new NativeArray<float>( 1, Allocator.TempJob );
        using var solid     = new NativeArray<bool>( 1, Allocator.TempJob );
        using var managed   = new NativeArray<bool>( 1, Allocator.TempJob );

        new SearchLineJob {
            densityFunctions = functions,
            start            = start,
            direction        = direction,
            range            = 0.8f,
            samples          = 4,
            iterations       = 11,
            found            = found,
            startIsSolid     = solid,
            managed          = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        startIsSolid = solid[0];

        return found[0];
    }

    // what a search of four samples over 0.8 of a cell and eleven halvings leaves: 0.2 / 2^11 of a cell
    private const float lineStep = 0.2f / 2048;

    // appendix A.1.3: the ray is sampled to find the first step in which the surface is crossed, and that step is halved.
    // The wall is crossed twice within the range, on the way in and on the way out, and the first is the one found
    [Test]
    public void aSearchAlongALineFindsTheNearestCrossingFromTheStartsSide( ) {
        var found = searchLine( new float2( 0.0f, 0.5f ), new float2( 1.0f, 0.0f ), out var startIsSolid );

        Assert.IsFalse( startIsSolid, "the start is outside the wall" );
        Assert.That( found, Is.InRange( 0.25f - lineStep - 1e-5f, 0.25f + 1e-5f ), "how far the wall is from the start" );

        // from inside the wall, the way out
        found = searchLine( new float2( 0.4f, 0.5f ), new float2( 1.0f, 0.0f ), out startIsSolid );

        Assert.IsTrue( startIsSolid, "the start is inside the wall" );
        Assert.That( found, Is.InRange( 0.15f - lineStep - 1e-5f, 0.15f + 1e-5f ), "how far the far side of the wall is from the start" );

        // at an angle the same wall is farther along the ray
        found = searchLine( new float2( 0.0f, 0.5f ), new float2( 0.6f, 0.8f ), out _ );

        Assert.That( found, Is.InRange( ( 0.25f / 0.6f ) - lineStep - 1e-5f, ( 0.25f / 0.6f ) + 1e-5f ), "how far the wall is from the start along a ray at an angle" );
    }

    // "the method positions the output as far [...] as possible": with no crossing in range the last step is halved,
    // which ends at the range
    [Test]
    public void aSearchAlongALineThatFindsNothingEndsAtItsRange( ) {
        var found = searchLine( new float2( 0.0f, 0.5f ), new float2( -1.0f, 0.0f ), out _ );

        Assert.That( found, Is.InRange( 0.8f - lineStep - 1e-5f, 0.8f ), "how far the search went" );
    }

    // every face of the grid that the contour crosses once, from one of its edges to another: the 1D points of those
    // two edges, in cells from the minimum of the face, and the 2D point between them
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SearchFacesJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

        public BurstGrid               grid;
        public BurstOccupancy.Searches searches;

        public NativeList<int4>   faces;  // the minimum of each face and the axis it faces along
        public NativeList<float2> first;
        public NativeList<float2> second;
        public NativeList<float2> found;
        public NativeArray<bool>  managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            var resolution = this.grid.resolution;

            for( var axis = 0; axis < 3; ++axis ) {
                var across = ( axis + 1 ) % 3;
                var other  = ( axis + 2 ) % 3;

                for( var index = 0; index < ( resolution + 1 ) * resolution * resolution; ++index ) {
                    var corner = int3.zero;

                    corner[axis]   = index / ( resolution * resolution );
                    corner[across] = ( index / resolution ) % resolution;
                    corner[other]  = index % resolution;

                    var solidCorners = 0;

                    for( var faceCorner = 0; faceCorner < 4; ++faceCorner ) {
                        var position = corner;

                        position[across] += faceCorner & 1;
                        position[other]  += faceCorner >> 1;

                        if( BurstOccupancy.isSolid( this.grid.positionOf( position ), this.densityFunctions ) ) {
                            solidCorners |= 1 << faceCorner;
                        }
                    }

                    // the four edges of the face: along the first of its axes at either end of the second, and the
                    // other way round, each between two of its corners
                    var points  = new float2x4( );
                    var crossed = 0;

                    for( var edge = 0; edge < 4; ++edge ) {
                        var along     = edge < 2 ? across : other;
                        var atMaximum = ( edge & 1 ) != 0;
                        var start     = corner;

                        start[edge < 2 ? other : across] += atMaximum ? 1 : 0;

                        var startCorner = edge < 2 ? ( atMaximum ? 2 : 0 ) : ( atMaximum ? 1 : 0 );
                        var endCorner   = startCorner + ( edge < 2 ? 1 : 2 );

                        var startIsSolid = ( solidCorners & ( 1 << startCorner ) ) != 0;
                        var endIsSolid   = ( solidCorners & ( 1 << endCorner ) )   != 0;

                        if( startIsSolid == endIsSolid ) {
                            continue;
                        }

                        var part = BurstOccupancy.searchEdge( this.grid, start, along, startIsSolid, this.densityFunctions, this.searches.edgeIterations );

                        points[crossed++] = edge < 2 ? new float2( part, atMaximum ? 1.0f : 0.0f ) : new float2( atMaximum ? 1.0f : 0.0f, part );
                    }

                    if( crossed != 2 ) {
                        continue;
                    }

                    this.faces.Add( new int4( corner, axis ) );
                    this.first.Add( points[0] );
                    this.second.Add( points[1] );
                    this.found.Add( BurstOccupancy.searchFace( this.grid, corner, axis, points[0], points[1], solidCorners, this.searches.faceSamples, this.searches.faceIterations, this.densityFunctions ) );
                }
            }
        }

    }

    private struct FaceSearch {

        public int3    corner;
        public int     axis;
        public Vector2 first;
        public Vector2 second;
        public Vector2 found;

    }

    private static readonly BurstGrid faceGrid = new( 16 );

    private static FaceSearch[] searchFaces( Surface surface ) {
        using var functions = BurstDensityFunction.snapshot( new DensityFunction[] { shapeOf( surface ) }, Allocator.TempJob );
        using var faces     = new NativeList<int4>( Allocator.TempJob );
        using var first     = new NativeList<float2>( Allocator.TempJob );
        using var second    = new NativeList<float2>( Allocator.TempJob );
        using var found     = new NativeList<float2>( Allocator.TempJob );
        using var managed   = new NativeArray<bool>( 1, Allocator.TempJob );

        new SearchFacesJob {
            densityFunctions = functions,
            grid             = faceGrid,
            searches         = new BurstOccupancy.Searches { edgeIterations = 15, faceSamples = 4, faceIterations = 11 },
            faces            = faces,
            first            = first,
            second           = second,
            found            = found,
            managed          = managed
        }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        var searches = new FaceSearch[faces.Length];

        for( var index = 0; index < searches.Length; ++index ) {
            searches[index] = new FaceSearch {
                corner = faces[index].xyz,
                axis   = faces[index].w,
                first  = first[index],
                second = second[index],
                found  = found[index]
            };
        }

        return searches;
    }

    // a point of a face, given in cells from its minimum, in voxelizer object space
    private static Vector toObjectSpace( FaceSearch search, Vector2 point ) {
        var position = ( Vector3 )faceGrid.positionOf( search.corner );
        var result   = new double[] { position.x, position.y, position.z };

        result[( search.axis + 1 ) % 3] += point.x / 16.0;
        result[( search.axis + 2 ) % 3] += point.y / 16.0;

        return new Vector( result[0], result[1], result[2] );
    }

    // a point in the frame of the box, along its own axes from its center
    private static double[] toBox( Surface surface, Vector point ) {
        var shape = shapeOf( surface );
        var frame = new DensityFunction.Frame( shape );

        var offset = point - new Vector( shape.origin );

        return new[] { Vector.dot( offset, new Vector( frame.xAxis ) ), Vector.dot( offset, new Vector( frame.yAxis ) ), Vector.dot( offset, new Vector( frame.zAxis ) ) };
    }

    // the face of the box a point of its surface lies on: the axis of the box it is at right angles to and the side
    private static ( int axis, int side ) faceOfBox( double[] local ) {
        var sizes = new double[] { extents.x, extents.y, extents.z };
        var axis  = Enumerable.Range( 0, 3 ).OrderBy( ( candidate ) => Math.Abs( Math.Abs( local[candidate] ) - sizes[candidate] ) ).First( );

        return ( axis, local[axis] > 0.0 ? 1 : -1 );
    }

    // the 2D points of the faces of the grid that a box crosses, sorted by what lemma 5.1 says of them. Where both 1D
    // points lie on the same face of the box, the crossing is straight. Where they lie on two faces that share an edge
    // of the box, and that edge passes through the face of the grid, the crossing bends once, at the edge. Other faces
    // of the grid see more of the box than the lemma is about
    private static ( double straight, double bent, int straightCount, int bentCount ) worstFacePointsOf( Surface surface ) {
        var sizes = new double[] { extents.x, extents.y, extents.z };

        var straight      = 0.0;
        var bent          = 0.0;
        var straightCount = 0;
        var bentCount     = 0;

        foreach( var search in searchFaces( surface ) ) {
            var first  = faceOfBox( toBox( surface, toObjectSpace( search, search.first ) ) );
            var second = faceOfBox( toBox( surface, toObjectSpace( search, search.second ) ) );

            if( first == second ) {
                ++straightCount;

                straight = Math.Max( straight, ( search.found - ( ( search.first + search.second ) * 0.5f ) ).magnitude );

                continue;
            }

            if( first.axis == second.axis ) {
                continue;
            }

            // the edge of the box that the two faces share runs along the third axis of the box. Where it passes
            // through the plane of the face of the grid is found by going along it from either side of that plane
            var third = 3 - first.axis - second.axis;

            double heightAt( double along ) {
                var local = new double[3];

                local[first.axis]  = first.side  * sizes[first.axis];
                local[second.axis] = second.side * sizes[second.axis];
                local[third]       = along;

                return inGridFace( surface, search, local ).height;
            }

            var from = heightAt( -sizes[third] );
            var to   = heightAt(  sizes[third] );

            if( ( from > 0.0 ) == ( to > 0.0 ) ) {
                continue;
            }

            var crossing = -sizes[third] + ( 2.0 * sizes[third] * ( from / ( from - to ) ) );

            // away from the ends of the edge, where a third face of the box comes into the face of the grid
            if( Math.Abs( crossing ) > sizes[third] - ( 1.0 / 16 ) ) {
                continue;
            }

            var corner = new double[3];

            corner[first.axis]  = first.side  * sizes[first.axis];
            corner[second.axis] = second.side * sizes[second.axis];
            corner[third]       = crossing;

            var expected = inGridFace( surface, search, corner );

            if( expected.x < 0.0 || expected.x > 1.0 || expected.y < 0.0 || expected.y > 1.0 ) {
                continue;
            }

            ++bentCount;

            bent = Math.Max( bent, Math.Sqrt( Math.Pow( search.found.x - expected.x, 2.0 ) + Math.Pow( search.found.y - expected.y, 2.0 ) ) );
        }

        return ( straight, bent, straightCount, bentCount );
    }

    // a point given in the frame of the box, in cells from the minimum of a face of the grid along its two axes, and how
    // far it is from the plane of that face
    private static ( double x, double y, double height ) inGridFace( Surface surface, FaceSearch search, double[] local ) {
        var shape = shapeOf( surface );
        var frame = new DensityFunction.Frame( shape );
        var axes  = new[] { frame.xAxis, frame.yAxis, frame.zAxis };

        var position = new double[] { shape.origin.x, shape.origin.y, shape.origin.z };

        for( var axis = 0; axis < 3; ++axis ) {
            position[0] += local[axis] * axes[axis].x;
            position[1] += local[axis] * axes[axis].y;
            position[2] += local[axis] * axes[axis].z;
        }

        var minimum = ( Vector3 )faceGrid.positionOf( search.corner );
        var cells   = new double[] { ( position[0] - minimum.x ) * 16.0, ( position[1] - minimum.y ) * 16.0, ( position[2] - minimum.z ) * 16.0 };

        return ( cells[( search.axis + 1 ) % 3], cells[( search.axis + 2 ) % 3], cells[search.axis] );
    }

    // lemma 5.1: where the surface crosses a face straight, the 2D point is the middle of the two 1D points, and where
    // it bends once, the 2D point is at the bend
    [Test]
    public void theFacePointOfAStraightCrossingIsTheMiddleAndThatOfABendIsTheBend( [Values( Surface.Box, Surface.TurnedBox )] Surface surface ) {
        var ( straight, bent, straightCount, bentCount ) = worstFacePointsOf( surface );

        Assert.Greater( straightCount, 100, "faces crossed straight" );
        Assert.Greater( bentCount,     20,  "faces crossed with a bend" );

        Assert.AreEqual( 0.0, straight, "how far the 2D point of a straight crossing is from the middle, at worst, in cells" );

        Assert.LessOrEqual( bent, facePointTolerance, "how far the 2D point of a bent crossing is from the bend, at worst, in cells" );
    }

    // the searches step by up to a ten-thousandth of a cell, and the two lines are drawn through points that far off.
    // The farthest a 2D point was from its bend is 0.00031 of a cell, on the turned box
    private const double facePointTolerance = 1e-3;

    // on a curved surface the 2D point lies between the curve and the point where the tangents at the two 1D points
    // meet: outside the sphere's circle in the plane of the face, by less than those tangents are
    [Test]
    public void theFacePointOfACurvedCrossingLiesBetweenTheCurveAndItsTangents( ) {
        var center = new Vector( origin );
        var lowest = double.PositiveInfinity;
        var worst  = double.NegativeInfinity;
        var count  = 0;

        foreach( var search in searchFaces( Surface.Sphere ) ) {
            var found  = toObjectSpace( search, search.found );
            var first  = toObjectSpace( search, search.first );
            var second = toObjectSpace( search, search.second );

            // the middle of the circle the sphere draws in the plane of the face, and its radius, in cells
            var middle = new[] { center.x, center.y, center.z };

            middle[search.axis] = new[] { found.x, found.y, found.z }[search.axis];

            var circle = new Vector( middle[0], middle[1], middle[2] );
            var height = center - circle;
            var around = Math.Sqrt( ( ( double )radius * radius ) - Vector.dot( height, height ) ) * 16.0;

            double fromCircle( Vector point ) {
                return Math.Sqrt( Vector.dot( point - circle, point - circle ) ) * 16.0;
            }

            // the tangents at two points of a circle meet at the radius over the cosine of half the angle between them
            var cosine   = Vector.dot( first - circle, second - circle ) / ( fromCircle( first ) * fromCircle( second ) / 256.0 );
            var tangents = around / Math.Sqrt( ( 1.0 + cosine ) / 2.0 );

            // 0 on the circle, 1 where the tangents meet
            var between = ( fromCircle( found ) - around ) / ( tangents - around );

            // a crossing of a hundredth of a cell is too short to tell the two apart
            if( tangents - around < 1e-3 ) {
                continue;
            }

            ++count;

            lowest = Math.Min( lowest, between );
            worst  = Math.Max( worst,  between );
        }

        Assert.Greater( count, 100, "faces crossed" );

        Assert.GreaterOrEqual( lowest, curveTolerance.lowest, "the 2D point nearest the middle of the circle, from the circle (0) to where the tangents meet (1)" );
        Assert.LessOrEqual(    worst,  curveTolerance.worst,  "the 2D point farthest from the middle of the circle, from the circle (0) to where the tangents meet (1)" );
    }

    // the 2D points of the sphere are between 0.002 and 0.27 of the way from the circle to where the tangents meet: far
    // nearer the curve than the tangents are
    private static readonly ( double lowest, double worst ) curveTolerance = ( 0.0, 0.5 );

    // the ways a 2D point falls back to the middle of its two 1D points, on the face at the middle of the grid, with
    // the wall of the tests above
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct SearchFaceJob : IJob {

        [ReadOnly] public NativeArray<BurstDensityFunction> densityFunctions;

        public float2 first;
        public float2 second;
        public int    solidCorners;

        public NativeArray<float2> found;
        public NativeArray<bool>   managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            this.found[0] = BurstOccupancy.searchFace( new BurstGrid( 16 ), new int3( 8, 8, 8 ), 2, this.first, this.second, this.solidCorners, 4, 11, this.densityFunctions );
        }

    }

    private static Vector2 searchFace( float2 first, float2 second, int solidCorners ) {
        using var functions = BurstDensityFunction.snapshot( new DensityFunction[] { wall }, Allocator.TempJob );
        using var found     = new NativeArray<float2>( 1, Allocator.TempJob );
        using var managed   = new NativeArray<bool>( 1, Allocator.TempJob );

        new SearchFaceJob { densityFunctions = functions, first = first, second = second, solidCorners = solidCorners, found = found, managed = managed }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        return found[0];
    }

    [Test]
    public void aFacePointFallsBackToTheMiddleOfItsTwoPoints( ) {
        // the two 1D points in the same place
        Assert.AreEqual( new Vector2( 0.25f, 0.5f ), searchFace( new float2( 0.25f, 0.5f ), new float2( 0.25f, 0.5f ), 0b0000 ), "of two points in the same place" );

        // on the near side of the wall, which crosses the face straight
        Assert.AreEqual( new Vector2( 0.25f, 0.5f ), searchFace( new float2( 0.25f, 0.0f ), new float2( 0.25f, 1.0f ), 0b0000 ), "of a straight crossing" );

        // far from any surface on the side the corners of the face point to: the first search finds nothing, the other
        // two neither, and lines that far apart do not meet beyond the top
        Assert.AreEqual( new Vector2( -2.0f, 0.5f ), searchFace( new float2( -2.0f, 0.2f ), new float2( -2.0f, 0.8f ), 0b0000 ), "of two points with no surface near them" );
    }

}
