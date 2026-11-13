using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// BurstQuadric against answers known in closed form: distances to planes and lines worked out one at a time, the point
// where planes meet, and the area-weighted centre of the vertices of a flat patch. The quadrics are built, added and
// solved in a Burst job, and what the tests expect is worked out here in doubles
public class BurstQuadricTest {

    // what a quadric is made of
    private enum Kind {
        Plane,     // a: unit normal, b: a point of the plane
        Triangle,  // a, b, c: its vertices
        Boundary,  // a, b: the ends of the edge, c: the vertex opposite to it
        Line       // a: a point of the line, b: its direction, of any length
    }

    private struct Item {

        public Kind    kind;
        public double3 a, b, c;
        public double  weight;

    }

    // a quadric: the sum of some items, each times its weight, and a point to take its error at
    private struct Case {

        public int     start;
        public int     count;
        public double3 query;

    }

    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct QuadricJob : IJob {

        [ReadOnly] public NativeArray<Item> items;
        [ReadOnly] public NativeArray<Case> cases;

        // for every item: whether it has a quadric, and the area of a triangle
        public NativeArray<bool>   made;
        public NativeArray<double> areas;

        // for every case: the error at its query, whether it has a point of least error, that point and the error there
        public NativeArray<double>  errors;
        public NativeArray<bool>    solved;
        public NativeArray<double3> best;
        public NativeArray<double>  bestErrors;

        public NativeArray<bool> managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var index = 0; index < this.cases.Length; ++index ) {
                var sum = default( BurstQuadric );

                for( var at = this.cases[index].start; at < this.cases[index].start + this.cases[index].count; ++at ) {
                    var item    = this.items[at];
                    var quadric = default( BurstQuadric );
                    var area    = 0.0;
                    var made    = true;

                    switch( item.kind ) {
                        case Kind.Plane:    quadric = BurstQuadric.plane( item.a, item.b );                               break;
                        case Kind.Triangle: made    = BurstQuadric.triangle( item.a, item.b, item.c, out quadric, out area ); break;
                        case Kind.Boundary: made    = BurstQuadric.boundary( item.a, item.b, item.c, out quadric );           break;
                        case Kind.Line:     made    = BurstQuadric.line( item.a, item.b, out quadric );                       break;
                    }

                    this.made[at]  = made;
                    this.areas[at] = area;

                    if( made ) {
                        sum += quadric * item.weight;
                    }
                }

                this.errors[index] = sum.evaluate( this.cases[index].query );
                this.solved[index] = sum.minimize( out var best );
                this.best[index]   = best;

                this.bestErrors[index] = this.solved[index] ? sum.evaluate( best ) : double.NaN;
            }
        }

    }

    // the cases of a test and what the job made of them
    private class Run {

        private readonly List<Item> items = new( );
        private readonly List<Case> cases = new( );

        public bool[]    made;
        public double[]  areas;
        public double[]  errors;
        public bool[]    solved;
        public double3[] best;
        public double[]  bestErrors;

        // adds a quadric and gives its index among the cases. Its items follow each other from firstItem( index ) on
        public int add( double3 query, params Item[] items ) {
            this.cases.Add( new Case { start = this.items.Count, count = items.Length, query = query } );
            this.items.AddRange( items );

            return this.cases.Count - 1;
        }

        public int firstItem( int index ) {
            return this.cases[index].start;
        }

        public void run( ) {
            using var items      = new NativeArray<Item>( this.items.ToArray( ), Allocator.TempJob );
            using var cases      = new NativeArray<Case>( this.cases.ToArray( ), Allocator.TempJob );
            using var made       = new NativeArray<bool>( items.Length, Allocator.TempJob );
            using var areas      = new NativeArray<double>( items.Length, Allocator.TempJob );
            using var errors     = new NativeArray<double>( cases.Length, Allocator.TempJob );
            using var solved     = new NativeArray<bool>( cases.Length, Allocator.TempJob );
            using var best       = new NativeArray<double3>( cases.Length, Allocator.TempJob );
            using var bestErrors = new NativeArray<double>( cases.Length, Allocator.TempJob );
            using var managed    = new NativeArray<bool>( 1, Allocator.TempJob );

            new QuadricJob {
                items      = items,
                cases      = cases,
                made       = made,
                areas      = areas,
                errors     = errors,
                solved     = solved,
                best       = best,
                bestErrors = bestErrors,
                managed    = managed
            }.Run( );

            BurstTestSupport.assertRanUnderBurst( managed );

            this.made       = made.ToArray( );
            this.areas      = areas.ToArray( );
            this.errors     = errors.ToArray( );
            this.solved     = solved.ToArray( );
            this.best       = best.ToArray( );
            this.bestErrors = bestErrors.ToArray( );
        }

    }

    private static Item plane( double3 normal, double3 point, double weight = 1.0 ) {
        return new Item { kind = Kind.Plane, a = normal, b = point, weight = weight };
    }

    private static Item triangle( double3 v0, double3 v1, double3 v2, double weight = 1.0 ) {
        return new Item { kind = Kind.Triangle, a = v0, b = v1, c = v2, weight = weight };
    }

    private static Item boundary( double3 start, double3 end, double3 opposite, double weight = 1.0 ) {
        return new Item { kind = Kind.Boundary, a = start, b = end, c = opposite, weight = weight };
    }

    private static Item line( double3 point, double3 direction, double weight = 1.0 ) {
        return new Item { kind = Kind.Line, a = point, b = direction, weight = weight };
    }

    private static double range( System.Random random, double minimum, double maximum ) {
        return minimum + ( random.NextDouble( ) * ( maximum - minimum ) );
    }

    // a point of the grid, which spans -0.5 to 0.5 on every axis
    private static double3 point( System.Random random ) {
        return new double3( range( random, -0.5, 0.5 ), range( random, -0.5, 0.5 ), range( random, -0.5, 0.5 ) );
    }

    private static double3 direction( System.Random random ) {
        while( true ) {
            var vector = new double3( range( random, -1.0, 1.0 ), range( random, -1.0, 1.0 ), range( random, -1.0, 1.0 ) );
            var length = math.length( vector );

            if( length > 0.1 && length < 1.0 ) {
                return vector / length;
            }
        }
    }

    // two unit vectors that are square to the unit normal and to each other, by the Gram-Schmidt process, as section 4.2
    // of Liu et al. makes them
    private static ( double3 x, double3 y ) squareTo( double3 normal ) {
        // the axis the normal leans on least
        var axis = math.abs( normal.x ) <= math.abs( normal.y ) && math.abs( normal.x ) <= math.abs( normal.z )
            ? new double3( 1.0, 0.0, 0.0 )
            : math.abs( normal.y ) <= math.abs( normal.z ) ? new double3( 0.0, 1.0, 0.0 ) : new double3( 0.0, 0.0, 1.0 );

        var x = math.normalize( axis - ( normal * math.dot( axis, normal ) ) );

        return ( x, math.cross( x, normal ) );
    }

    private static double squared( double value ) {
        return value * value;
    }

    private static void assertClose( double expected, double actual, double tolerance, string what ) {
        Assert.AreEqual( expected, actual, tolerance * Math.Max( 1.0, Math.Abs( expected ) ), what );
    }

    private static void assertClose( double3 expected, double3 actual, double tolerance, string what ) {
        Assert.LessOrEqual( math.length( expected - actual ), tolerance, $"{what}: expected {expected}, was {actual}" );
    }

    // Garland and Heckbert, equation 2: the error of a point is the sum of its squared distances to the planes, here
    // each times a weight
    [Test]
    public void errorIsTheSumOfSquaredDistancesToThePlanes( ) {
        var random   = new System.Random( 1 );
        var run      = new Run( );
        var expected = new List<double>( );

        for( var index = 0; index < 300; ++index ) {
            var query = point( random );
            var items = new Item[1 + random.Next( 8 )];
            var sum   = 0.0;

            for( var at = 0; at < items.Length; ++at ) {
                var normal = direction( random );
                var on     = point( random );
                var weight = range( random, 0.1, 3.0 );

                items[at] = plane( normal, on, weight );

                sum += weight * squared( math.dot( normal, query - on ) );
            }

            run.add( query, items );
            expected.Add( sum );
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            assertClose( expected[index], run.errors[index], 1e-12, $"error of quadric {index}" );
        }
    }

    // a quadric holds errors far smaller than the squares of the coordinates they are worked out from: a millionth of a
    // unit from a plane that is most of a unit from the origin
    [Test]
    public void smallErrorsFarFromTheOriginAreKept( ) {
        var random   = new System.Random( 2 );
        var run      = new Run( );
        var expected = new List<double>( );

        for( var index = 0; index < 300; ++index ) {
            var normal   = direction( random );
            var on       = direction( random ) * range( random, 0.6, 0.85 );
            var ( x, y ) = squareTo( normal );
            var distance = range( random, 0.5e-6, 2e-6 );

            // off the plane by the distance, and anywhere along it
            var query = on + ( normal * distance ) + ( x * range( random, -0.3, 0.3 ) ) + ( y * range( random, -0.3, 0.3 ) );

            run.add( query, plane( normal, on ) );
            expected.Add( squared( math.dot( normal, query - on ) ) );
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            Assert.AreEqual( expected[index], run.errors[index], 0.01 * expected[index], $"error of quadric {index}" );
        }
    }

    // Liu et al., equation 8: the quadric of a triangle is that of its plane, weighted by its area
    [Test]
    public void triangleQuadricIsTheSquaredDistanceToItsPlane( ) {
        var random   = new System.Random( 3 );
        var run      = new Run( );
        var expected = new List<( double error, double area )>( );

        for( var index = 0; index < 300; ++index ) {
            var v0 = point( random );
            var v1 = point( random );
            var v2 = point( random );

            var normal = math.cross( v1 - v0, v2 - v0 );
            var area   = 0.5 * math.length( normal );

            normal = math.normalize( normal );

            // the corners and a point of the plane beyond them are on the plane, the last is anywhere
            var inPlane = v0 + ( ( v1 - v0 ) * range( random, -2.0, 2.0 ) ) + ( ( v2 - v0 ) * range( random, -2.0, 2.0 ) );
            var queries = new[] { v0, v1, v2, inPlane, point( random ) };

            foreach( var query in queries ) {
                run.add( query, triangle( v0, v1, v2 ) );
                expected.Add( ( squared( math.dot( normal, query - v0 ) ), area ) );
            }
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            Assert.IsTrue( run.made[run.firstItem( index )], $"triangle {index} has a plane" );

            assertClose( expected[index].area,  run.areas[run.firstItem( index )], 1e-12, $"area of triangle {index}" );
            assertClose( expected[index].error, run.errors[index],                 1e-12, $"error of triangle {index}" );
        }
    }

    [Test]
    public void triangleWithoutAreaHasNoPlane( ) {
        var run = new Run( );

        var corner = new double3( 1.0, 2.0, 3.0 );
        var step   = new double3( 1.0, -2.0, 0.5 );

        // a vertex used twice, all three the same, and three in a row
        run.add( double3.zero, triangle( corner, corner, corner + step ) );
        run.add( double3.zero, triangle( corner, corner + step, corner ) );
        run.add( double3.zero, triangle( corner, corner, corner ) );
        run.add( double3.zero, triangle( corner, corner + step, corner + ( step * 2.0 ) ) );

        run.run( );

        for( var index = 0; index < 4; ++index ) {
            Assert.IsFalse( run.made[run.firstItem( index )], $"triangle {index} has a plane" );

            Assert.AreEqual( 0.0, run.areas[run.firstItem( index )], $"area of triangle {index}" );
            Assert.AreEqual( 0.0, run.errors[index],                 $"error of triangle {index}" );
        }
    }

    // Liu et al., section 4.1: a line quadric is the squared distance to the line through a vertex along its normal,
    // and is the sum of two plane quadrics through the line (equations 13 to 15)
    [Test]
    public void lineQuadricIsTheSquaredDistanceToItsLine( ) {
        var random   = new System.Random( 4 );
        var run      = new Run( );
        var expected = new List<double>( );
        var planes   = new List<double>( );

        for( var index = 0; index < 300; ++index ) {
            var on     = point( random );
            var normal = direction( random );
            var query  = index % 3 == 0 ? on + ( normal * range( random, -1.0, 1.0 ) ) : point( random );

            // the direction is given at any length
            run.add( query, line( on, normal * range( random, 0.2, 5.0 ) ) );

            expected.Add( math.lengthsq( math.cross( query - on, normal ) ) );

            var ( x, y ) = squareTo( normal );

            planes.Add( squared( math.dot( x, query - on ) ) + squared( math.dot( y, query - on ) ) );
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            Assert.IsTrue( run.made[run.firstItem( index )], $"line {index} has a quadric" );

            assertClose( expected[index], run.errors[index], 1e-12, $"squared distance to line {index}" );
            assertClose( planes[index],   run.errors[index], 1e-12, $"sum of the two plane quadrics of line {index}" );
        }
    }

    [Test]
    public void lineWithoutDirectionHasNoQuadric( ) {
        var run = new Run( );

        run.add( new double3( 0.3, 0.2, 0.1 ), line( new double3( 0.1, 0.2, 0.3 ), double3.zero ) );
        run.run( );

        Assert.IsFalse( run.made[0], "the line has a quadric" );

        Assert.AreEqual( 0.0, run.errors[0], "error" );
    }

    // Garland and Heckbert, section 6: the plane added for a triangle of an edge that is to keep its shape runs through
    // the edge and is perpendicular to the triangle
    [Test]
    public void boundaryPlaneHoldsItsEdgeAndIsSquareToItsTriangle( ) {
        var random   = new System.Random( 5 );
        var run      = new Run( );
        var expected = new List<double>( );

        for( var index = 0; index < 300; ++index ) {
            var start    = point( random );
            var end      = point( random );
            var opposite = point( random );

            var along  = math.normalize( end - start );
            var normal = math.normalize( math.cross( end - start, opposite - start ) );

            // how far the opposite vertex is from the line of the edge
            var height = math.length( math.cross( opposite - start, along ) );

            // on the plane: the ends, anywhere on the line of the edge, and from there any way along the normal of the
            // triangle. The opposite vertex is its height away, and so is any point at that height
            var onEdge  = start + ( ( end - start ) * range( random, -2.0, 2.0 ) );
            var lift    = range( random, -1.0, 1.0 );
            var inward  = math.cross( normal, along );
            var queries = new[] {
                ( start,                                    0.0 ),
                ( end,                                      0.0 ),
                ( onEdge,                                   0.0 ),
                ( onEdge + ( normal * lift ),               0.0 ),
                ( opposite,                                 squared( height ) ),
                ( onEdge + ( normal * lift ) + ( inward * 0.25 ), squared( 0.25 ) )
            };

            foreach( var ( query, error ) in queries ) {
                run.add( query, boundary( start, end, opposite ) );
                expected.Add( error );
            }
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            Assert.IsTrue( run.made[run.firstItem( index )], $"edge {index} has a plane" );

            assertClose( expected[index], run.errors[index], 1e-10, $"error of the plane of edge {index}" );
        }
    }

    [Test]
    public void boundaryOfAnEdgeWithoutLengthOrATriangleWithoutAreaHasNoPlane( ) {
        var run = new Run( );

        var corner = new double3( 1.0, 2.0, 3.0 );
        var step   = new double3( 1.0, -2.0, 0.5 );

        run.add( double3.zero, boundary( corner, corner, corner + step ) );
        run.add( double3.zero, boundary( corner, corner + step, corner + ( step * 2.0 ) ) );
        run.add( double3.zero, boundary( corner, corner + step, corner ) );

        run.run( );

        for( var index = 0; index < 3; ++index ) {
            Assert.IsFalse( run.made[run.firstItem( index )], $"edge {index} has a plane" );
        }
    }

    // Garland and Heckbert, section 5: a vertex is the solution of the intersection of the planes of its triangles, and
    // the point of least error of planes that meet in one point is that point
    [Test]
    public void bestPointOfPlanesThatMeetIsWhereTheyMeet( ) {
        var random   = new System.Random( 6 );
        var run      = new Run( );
        var expected = new List<double3>( );

        for( var index = 0; index < 300; ++index ) {
            var meeting = point( random );
            var items   = new Item[3 + random.Next( 4 )];

            // the first three planes are far from parallel to each other
            double3 n0, n1, n2;
            do {
                n0 = direction( random );
                n1 = direction( random );
                n2 = direction( random );
            }
            while( math.abs( math.dot( n0, math.cross( n1, n2 ) ) ) < 0.3 );

            for( var at = 0; at < items.Length; ++at ) {
                var normal   = at == 0 ? n0 : at == 1 ? n1 : at == 2 ? n2 : direction( random );
                var ( x, y ) = squareTo( normal );

                // the plane is given by any of its points
                items[at] = plane( normal, meeting + ( x * range( random, -1.0, 1.0 ) ) + ( y * range( random, -1.0, 1.0 ) ), range( random, 0.1, 3.0 ) );
            }

            run.add( double3.zero, items );
            expected.Add( meeting );
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            Assert.IsTrue( run.solved[index], $"quadric {index} has a best point" );

            assertClose( expected[index], run.best[index], 1e-11, $"best point of quadric {index}" );

            Assert.AreEqual( 0.0, run.bestErrors[index], 1e-13, $"error at the best point of quadric {index}" );
        }
    }

    // where the error is least its gradient is zero: 2 ( A x + b ), the sum over the planes of twice their weight times
    // their distance to x times their normal
    [Test]
    public void bestPointIsWhereTheGradientOfTheErrorIsZero( ) {
        var random = new System.Random( 7 );
        var run    = new Run( );
        var planes = new List<Item[]>( );

        for( var index = 0; index < 300; ++index ) {
            var items = new Item[4 + random.Next( 6 )];

            for( var at = 0; at < items.Length; ++at ) {
                items[at] = plane( direction( random ), point( random ), range( random, 0.1, 3.0 ) );
            }

            run.add( double3.zero, items );
            planes.Add( items );
        }

        run.run( );

        for( var index = 0; index < planes.Count; ++index ) {
            Assert.IsTrue( run.solved[index], $"quadric {index} has a best point" );

            var gradient = double3.zero;
            var error    = 0.0;

            foreach( var item in planes[index] ) {
                var distance = math.dot( item.a, run.best[index] - item.b );

                gradient += item.a * ( 2.0 * item.weight * distance );
                error    += item.weight * squared( distance );
            }

            Assert.LessOrEqual( math.length( gradient ), 1e-10, $"gradient at the best point of quadric {index}" );

            assertClose( error, run.bestErrors[index], 1e-12, $"error at the best point of quadric {index}" );
        }
    }

    // Garland and Heckbert, section 5.1: the planes around a flat region are parallel and those around a straight crease
    // are parallel to a line, and neither kind meets in a single point
    [Test]
    public void planesThatDoNotMeetInOnePointHaveNoBestPoint( ) {
        var random = new System.Random( 8 );
        var run    = new Run( );

        for( var index = 0; index < 100; ++index ) {
            var normal   = direction( random );
            var ( x, y ) = squareTo( normal );

            // nothing at all
            run.add( double3.zero );

            // one plane, and the same plane through several of its points
            run.add( double3.zero, plane( normal, point( random ) ) );

            var on = point( random );
            run.add( double3.zero, plane( normal, on ), plane( normal, on + ( x * 0.3 ) ), plane( normal, on + ( y * 0.2 ) - ( x * 0.1 ) ) );

            // parallel planes
            run.add( double3.zero, plane( normal, point( random ) ), plane( -normal, point( random ) ), plane( normal, point( random ) ) );

            // planes along a line: every normal is square to it
            var crease = new Item[2 + random.Next( 4 )];
            for( var at = 0; at < crease.Length; ++at ) {
                var angle = range( random, 0.0, 2.0 * math.PI_DBL );

                crease[at] = plane( ( x * math.cos( angle ) ) + ( y * math.sin( angle ) ), on, range( random, 0.1, 3.0 ) );
            }
            run.add( double3.zero, crease );
        }

        // creases that run almost along an axis of the grid. The entry of A on the diagonal for that axis is then far
        // smaller than the other two, and what is left for the last pivot has to be held against the largest of them
        for( var axis = 0; axis < 3; ++axis ) {
            for( var index = 0; index < 50; ++index ) {
                var along = double3.zero;
                along[axis] = 1.0;
                along = math.normalize( along + ( direction( random ) * 1e-5 ) );

                var ( x, y ) = squareTo( along );
                var on       = point( random );

                var crease = new Item[2 + random.Next( 4 )];
                for( var at = 0; at < crease.Length; ++at ) {
                    var angle = range( random, 0.0, 2.0 * math.PI_DBL );

                    crease[at] = plane( ( x * math.cos( angle ) ) + ( y * math.sin( angle ) ), on, range( random, 0.1, 3.0 ) );
                }
                run.add( double3.zero, crease );
            }
        }

        run.run( );

        for( var index = 0; index < run.solved.Length; ++index ) {
            Assert.IsFalse( run.solved[index], $"quadric {index} has a best point" );
        }
    }

    // two planes that are square to each other and a third at a small angle to the first. The smallest pivot of A is
    // then about a quarter of the angle squared against the largest: 2.5e-9 at a ten-thousandth of a radian, which is
    // solved, and 2.5e-13 at a millionth, which is under BurstQuadric.PivotLimit and is not
    [Test]
    public void planesThatAlmostDoNotMeetHaveNoBestPoint( ) {
        var random  = new System.Random( 9 );
        var run     = new Run( );
        var meeting = new List<double3>( );

        foreach( var angle in new[] { 1e-4, 1e-6 } ) {
            for( var index = 0; index < 100; ++index ) {
                var normal   = direction( random );
                var ( x, y ) = squareTo( normal );
                var on       = point( random );

                run.add( double3.zero, plane( normal, on ), plane( x, on ), plane( ( normal * math.cos( angle ) ) + ( y * math.sin( angle ) ), on ) );
                meeting.Add( on );
            }
        }

        run.run( );

        for( var index = 0; index < 100; ++index ) {
            Assert.IsTrue( run.solved[index], $"quadric {index}, at a ten-thousandth of a radian, has a best point" );

            assertClose( meeting[index], run.best[index], 1e-6, $"best point of quadric {index}" );
        }
        for( var index = 100; index < 200; ++index ) {
            Assert.IsFalse( run.solved[index], $"quadric {index}, at a millionth of a radian, has a best point" );
        }
    }

    // Liu et al., lemma 4.1: on a flat region the plane quadrics of the vertices are all the same plane, and with line
    // quadrics of the weight times each vertex's area the point of least error is the centre of the vertices weighted by
    // their areas, whatever the weight
    [Test]
    public void flatPatchWithLineQuadricsHasItsBestPointAtTheAreaWeightedCentre( [Values( 1e-4, 1e-3, 1e-1 )] double weight ) {
        var random   = new System.Random( 10 );
        var run      = new Run( );
        var expected = new List<double3>( );

        for( var index = 0; index < 300; ++index ) {
            var normal   = direction( random );
            var ( x, y ) = squareTo( normal );
            var on       = point( random );

            var count   = 2 + random.Next( 9 );
            var items   = new List<Item>( );
            var planes  = new List<Item>( );
            var centre  = double3.zero;
            var areaSum = 0.0;

            for( var vertex = 0; vertex < count; ++vertex ) {
                var position = on + ( x * range( random, -0.2, 0.2 ) ) + ( y * range( random, -0.2, 0.2 ) );
                var area     = range( random, 0.1, 2.0 );

                planes.Add( plane( normal, position, area ) );

                items.Add( plane( normal, position, area ) );
                items.Add( line( position, normal, weight * area ) );

                centre  += position * area;
                areaSum += area;
            }

            run.add( double3.zero, items.ToArray( ) );
            expected.Add( centre / areaSum );

            // without the lines there is no such point
            run.add( double3.zero, planes.ToArray( ) );
        }

        run.run( );

        for( var index = 0; index < expected.Count; ++index ) {
            Assert.IsTrue( run.solved[index * 2], $"patch {index} has a best point" );

            assertClose( expected[index], run.best[index * 2], 1e-10, $"best point of patch {index}" );

            Assert.IsFalse( run.solved[( index * 2 ) + 1], $"patch {index} without line quadrics has a best point" );
        }
    }

}
