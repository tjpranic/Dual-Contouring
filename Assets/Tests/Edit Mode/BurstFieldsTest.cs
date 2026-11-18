using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;
using Symmetry           = BurstFields.Symmetry;

// the two fields of Instant Field-Aligned Meshes (BurstFields; Documents/Plans/Remeshing.md, step 4): the operators
// against searches over everything they choose from, the mesh the fields are on, the hierarchy and its colours, and
// the fields themselves on shapes where they are known in closed form
public class BurstFieldsTest : TopologyTest {

    // ---- the operators ----

    private struct Case {

        public float3 v0, n0, q0, o0;
        public float3 v1, n1, q1, o1;
        public float  rho;

    }

    private struct Answer {

        public int2   turns;
        public float3 a, b;
        public float3 between;
        public int2   rounded;
        public int2   steps0, steps1;
        public float3 p0, p1;

    }

    // every operator on every case, in a job compiled by Burst
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    private struct OperatorsJob : IJob {

        public Symmetry symmetry;

        [ReadOnly] public NativeArray<Case> cases;

        public NativeArray<Answer> answers;
        public NativeArray<bool>   managed;

        public void Execute( ) {
            this.managed[0] = BurstTestSupport.runningManaged( );

            for( var index = 0; index < this.cases.Length; ++index ) {
                var c      = this.cases[index];
                var answer = new Answer( );

                answer.turns   = BurstFields.matchOrientations( c.q0, c.n0, c.q1, c.n1, this.symmetry, out answer.a, out answer.b );
                answer.between = BurstFields.middle( c.v0, c.n0, c.v1, c.n1 );
                answer.rounded = BurstFields.roundSteps( c.o0, c.q0, BurstFields.secondAxis( c.q0, c.n0, this.symmetry ), c.v0, c.rho, this.symmetry );

                BurstFields.matchPositions( c.v0, c.n0, c.q0, c.o0, c.v1, c.n1, c.q1, c.o1, c.rho, this.symmetry, out answer.steps0, out answer.steps1, out answer.p0, out answer.p1 );

                this.answers[index] = answer;
            }
        }

    }

    private static double3 d( float3 value ) {
        return new double3( value.x, value.y, value.z );
    }

    private static double3 direction( System.Random random ) {
        while( true ) {
            var vector = new double3( ( random.NextDouble( ) * 2.0 ) - 1.0, ( random.NextDouble( ) * 2.0 ) - 1.0, ( random.NextDouble( ) * 2.0 ) - 1.0 );
            var length = math.length( vector );

            if( length > 0.1 && length <= 1.0 ) {
                return vector / length;
            }
        }
    }

    // a direction at right angles to a normal
    private static double3 tangent( System.Random random, double3 normal ) {
        while( true ) {
            var vector = direction( random );

            vector -= normal * math.dot( normal, vector );

            if( math.length( vector ) > 0.1 ) {
                return math.normalize( vector );
            }
        }
    }

    // a vector turned about an axis of unit length by an angle
    private static double3 rotated( double3 vector, double3 axis, double angle ) {
        return ( vector * math.cos( angle ) ) + ( math.cross( axis, vector ) * math.sin( angle ) ) + ( axis * math.dot( axis, vector ) * ( 1.0 - math.cos( angle ) ) );
    }

    // two vertices as neighbours on a surface are: within a length of each other, their normals within the angle, each
    // with a direction in its tangent plane and a lattice point in it near the vertex
    private static Case[] cases( int count, int seed, double mostDegrees ) {
        var random = new System.Random( seed );
        var cases  = new Case[count];

        for( var index = 0; index < count; ++index ) {
            var rho = 0.5 + random.NextDouble( );
            var n0  = direction( random );
            var n1  = math.normalize( rotated( n0, tangent( random, n0 ), math.radians( mostDegrees * random.NextDouble( ) ) ) );
            var q0  = tangent( random, n0 );
            var q1  = tangent( random, n1 );
            var v0  = direction( random ) * 3.0 * random.NextDouble( );
            var v1  = v0 + ( direction( random ) * rho * random.NextDouble( ) );
            var o0  = v0 + ( ( q0 * ( random.NextDouble( ) - 0.5 ) ) + ( math.cross( n0, q0 ) * ( random.NextDouble( ) - 0.5 ) ) ) * rho;
            var o1  = v1 + ( ( q1 * ( random.NextDouble( ) - 0.5 ) ) + ( math.cross( n1, q1 ) * ( random.NextDouble( ) - 0.5 ) ) ) * rho;

            cases[index] = new Case {
                v0 = ( float3 )v0, n0 = ( float3 )n0, q0 = ( float3 )q0, o0 = ( float3 )o0,
                v1 = ( float3 )v1, n1 = ( float3 )n1, q1 = ( float3 )q1, o1 = ( float3 )o1,
                rho = ( float )rho
            };
        }

        return cases;
    }

    private static Answer[] answersOf( Case[] cases, Symmetry symmetry ) {
        using var nativeCases = new NativeArray<Case>( cases, Allocator.TempJob );
        using var answers     = new NativeArray<Answer>( cases.Length, Allocator.TempJob );
        using var managed     = new NativeArray<bool>( 1, Allocator.TempJob );

        new OperatorsJob { symmetry = symmetry, cases = nativeCases, answers = answers, managed = managed }.Run( );

        BurstTestSupport.assertRanUnderBurst( managed );

        return answers.ToArray( );
    }

    [Test]
    public void aTurnIsAStepOfTheSymmetryAboutTheNormal( [Values] Symmetry symmetry ) {
        var random = new System.Random( 11 );
        var steps  = ( int )symmetry;

        for( var index = 0; index < 50; ++index ) {
            var normal  = direction( random );
            var tangent = BurstFieldsTest.tangent( random, normal );

            for( var turns = -steps; turns <= 2 * steps; ++turns ) {
                var expected = rotated( tangent, normal, turns * 2.0 * math.PI / steps );
                var actual   = d( BurstFields.turned( ( float3 )tangent, ( float3 )normal, turns, symmetry ) );

                Assert.Less( math.distance( expected, actual ), 1e-5, $"{turns} turns of {steps}" );
            }
        }
    }

    [Test]
    public void theTurnsOfTwoDirectionsAreTheNearestOfAll( [Values] Symmetry symmetry ) {
        var cases   = BurstFieldsTest.cases( 2000, 21, 120.0 );
        var answers = answersOf( cases, symmetry );
        var steps   = ( int )symmetry;

        for( var index = 0; index < cases.Length; ++index ) {
            var c      = cases[index];
            var answer = answers[index];

            // every pair of turns
            var best = double.NegativeInfinity;

            for( var k = 0; k < steps; ++k )
            for( var l = 0; l < steps; ++l ) {
                best = math.max( best, math.dot( rotated( d( c.q0 ), d( c.n0 ), k * 2.0 * math.PI / steps ), rotated( d( c.q1 ), d( c.n1 ), l * 2.0 * math.PI / steps ) ) );
            }

            Assert.AreEqual( best, math.dot( d( answer.a ), d( answer.b ) ), 1e-5, $"case {index}: the two are as near as any turns bring them" );

            Assert.Less( math.distance( rotated( d( c.q0 ), d( c.n0 ), answer.turns.x * 2.0 * math.PI / steps ), d( answer.a ) ), 1e-5, $"case {index}: the first, turned {answer.turns.x}" );
            Assert.Less( math.distance( rotated( d( c.q1 ), d( c.n1 ), answer.turns.y * 2.0 * math.PI / steps ), d( answer.b ) ), 1e-5, $"case {index}: the second, turned {answer.turns.y}" );

            Assert.IsTrue( answer.turns.x >= 0 && answer.turns.x < steps && answer.turns.y >= 0 && answer.turns.y < steps, $"case {index}: turns within the symmetry" );
        }
    }

    [Test]
    public void thePointBetweenTwoVerticesIsInBothTangentPlanesAndTheNearestThatIs( ) {
        var cases   = BurstFieldsTest.cases( 2000, 31, 150.0 );
        var answers = answersOf( cases, Symmetry.Quads );
        var looked  = 0;

        for( var index = 0; index < cases.Length; ++index ) {
            var c       = cases[index];
            var between = d( answers[index].between );
            var cosine  = math.dot( d( c.n0 ), d( c.n1 ) );
            var middle  = ( d( c.v0 ) + d( c.v1 ) ) * 0.5;

            // planes at less than 20 degrees are the epsilon's, which brings the point to the middle of the two
            if( math.abs( cosine ) > math.cos( math.radians( 20.0 ) ) ) {
                continue;
            }

            ++looked;

            // the nearest point of both planes to the two vertices is the nearest to their middle: the middle moved
            // along the two normals
            var system   = new double2x2( 1.0, cosine, cosine, 1.0 );
            var along    = math.mul( math.inverse( system ), new double2( math.dot( d( c.n0 ), d( c.v0 ) - middle ), math.dot( d( c.n1 ), d( c.v1 ) - middle ) ) );
            var expected = middle + ( d( c.n0 ) * along.x ) + ( d( c.n1 ) * along.y );
            var apart    = math.distance( d( c.v0 ), d( c.v1 ) );

            // the epsilon of 1e-4 under the fraction moves the point by no more than this: each of the two steps along
            // the normals is at most 4 |v1 - v0| / (1 - cos^2), and comes out short by 1e-4 / (1 - cos^2) of itself
            var sine      = 1.0 - ( cosine * cosine );
            var tolerance = ( 2e-4 * apart / ( sine * sine ) ) + 1e-5;

            Assert.Less( math.abs( math.dot( d( c.n0 ), between - d( c.v0 ) ) ), tolerance, $"case {index}: in the first tangent plane" );
            Assert.Less( math.abs( math.dot( d( c.n1 ), between - d( c.v1 ) ) ), tolerance, $"case {index}: in the second tangent plane" );
            Assert.Less( math.distance( expected, between ),                     tolerance, $"case {index}: the nearest such point" );
        }

        Assert.Greater( looked, 1000, "cases with planes apart" );

        // one plane: the middle of the two
        var flat = new Case { v0 = new float3( 1.0f, 2.0f, 3.0f ), n0 = new float3( 0.0f, 0.0f, 1.0f ), v1 = new float3( 2.0f, 4.0f, 3.0f ), n1 = new float3( 0.0f, 0.0f, 1.0f ), rho = 1.0f };

        flat.q0 = flat.q1 = new float3( 1.0f, 0.0f, 0.0f );

        Assert.Less( math.distance( new float3( 1.5f, 3.0f, 3.0f ), answersOf( new[] { flat }, Symmetry.Quads )[0].between ), 1e-5f, "in one plane" );
    }

    // the lattice of a vertex in doubles: the point so many steps along its two axes
    private static double3 latticePoint( float3 o, float3 q, float3 n, int i, int j, double rho, Symmetry symmetry ) {
        var first  = d( q );
        var second = rotated( first, d( n ), 2.0 * math.PI / ( int )symmetry );

        return d( o ) + ( ( ( first * i ) + ( second * j ) ) * rho );
    }

    // the lattice points of a vertex within a few steps of a point, nearest first
    private static List<( double distance, double3 point )> latticeNear( float3 o, float3 q, float3 n, double3 point, double rho, Symmetry symmetry ) {
        var first  = d( q );
        var second = rotated( first, d( n ), 2.0 * math.PI / ( int )symmetry );
        var cosine = math.dot( first, second );
        var along  = new double2( math.dot( first, point - d( o ) ), math.dot( second, point - d( o ) ) ) / rho;
        var centre = new int2( ( int )math.round( ( along.x - ( cosine * along.y ) ) / ( 1.0 - ( cosine * cosine ) ) ), ( int )math.round( ( along.y - ( cosine * along.x ) ) / ( 1.0 - ( cosine * cosine ) ) ) );
        var result = new List<( double, double3 )>( );

        for( var i = -5; i <= 5; ++i )
        for( var j = -5; j <= 5; ++j ) {
            var candidate = latticePoint( o, q, n, centre.x + i, centre.y + j, rho, symmetry );

            result.Add( ( math.distance( candidate, point ), candidate ) );
        }

        result.Sort( ( a, b ) => a.Item1.CompareTo( b.Item1 ) );

        return result;
    }

    [Test]
    public void roundingGivesTheNearestPointOfTheLattice( [Values] Symmetry symmetry ) {
        var cases   = BurstFieldsTest.cases( 2000, 41, 60.0 );
        var answers = answersOf( cases, symmetry );

        for( var index = 0; index < cases.Length; ++index ) {
            var c       = cases[index];
            var rounded = latticePoint( c.o0, c.q0, c.n0, answers[index].rounded.x, answers[index].rounded.y, c.rho, symmetry );
            var nearest = latticeNear( c.o0, c.q0, c.n0, d( c.v0 ), c.rho, symmetry )[0];

            Assert.AreEqual( nearest.distance, math.distance( rounded, d( c.v0 ) ), 1e-4 * c.rho, $"case {index}" );
        }
    }

    [Test]
    public void theLatticeStepsOfTwoVerticesAreTheNearestOfThoseRoundThePointBetweenThem( [Values] Symmetry symmetry ) {
        var cases   = BurstFieldsTest.cases( 2000, 51, 60.0 );
        var answers = answersOf( cases, symmetry );

        for( var index = 0; index < cases.Length; ++index ) {
            var c       = cases[index];
            var answer  = answers[index];
            var between = d( answer.between );

            // the points of each lattice that the search looks at: for quads the corners of the cell the point between
            // lies in, and for triangles the six nearest it
            List<double3> near( float3 o, float3 q, float3 n ) {
                if( symmetry == Symmetry.Triangles ) {
                    return latticeNear( o, q, n, between, c.rho, symmetry ).Take( 6 ).Select( ( item ) => item.point ).ToList( );
                }

                var first  = d( q );
                var second = math.cross( d( n ), first );
                var corner = new int2( ( int )math.floor( math.dot( first, between - d( o ) ) / c.rho ), ( int )math.floor( math.dot( second, between - d( o ) ) / c.rho ) );

                return new List<double3> {
                    latticePoint( o, q, n, corner.x, corner.y, c.rho, symmetry ),     latticePoint( o, q, n, corner.x + 1, corner.y, c.rho, symmetry ),
                    latticePoint( o, q, n, corner.x, corner.y + 1, c.rho, symmetry ), latticePoint( o, q, n, corner.x + 1, corner.y + 1, c.rho, symmetry )
                };
            }

            var best = double.PositiveInfinity;

            foreach( var p in near( c.o0, c.q0, c.n0 ) )
            foreach( var q in near( c.o1, c.q1, c.n1 ) ) {
                best = math.min( best, math.distance( p, q ) );
            }

            Assert.AreEqual( best, math.distance( d( answer.p0 ), d( answer.p1 ) ), 1e-4 * c.rho, $"case {index}: the two are as near as any of them" );

            Assert.Less( math.distance( latticePoint( c.o0, c.q0, c.n0, answer.steps0.x, answer.steps0.y, c.rho, symmetry ), d( answer.p0 ) ), 1e-4 * c.rho, $"case {index}: the first is its steps from its lattice point" );
            Assert.Less( math.distance( latticePoint( c.o1, c.q1, c.n1, answer.steps1.x, answer.steps1.y, c.rho, symmetry ), d( answer.p1 ) ), 1e-4 * c.rho, $"case {index}: the second is its steps from its lattice point" );
        }
    }

    // ---- the mesh the fields are on, and level 0 ----

    private const double featureAngle = 40.0;

    private static BurstFields.Settings settingsOf( Symmetry symmetry, double length, bool creases = true, bool smallerRatio = false, bool measure = false ) {
        return new BurstFields.Settings {
            symmetry      = symmetry,
            length        = length,
            featureCosine = math.cos( math.radians( featureAngle ) ),
            creases       = creases,
            smallerRatio  = smallerRatio,
            iterations    = BurstFields.iterationsALevel,
            seed          = BurstFields.seed,
            measure       = measure
        };
    }

    // the fields of a mesh built by hand
    private sealed class Solved : IDisposable {

        public readonly BurstFields.Fields                  fields;
        public readonly BurstFields.Report                  report;
        public readonly BurstFields.Settings                settings;
        public readonly NativeList<BurstFields.LevelReport> levels = new( Allocator.TempJob );

        public Solved( HandBuiltMesh built, in BurstFields.Settings settings ) {
            using var positions = built.nativePositions( );
            using var normals   = built.nativeNormals( );
            using var triangles = built.nativeTriangles( );

            this.settings = settings;
            this.fields   = BurstFields.solve( positions, normals, triangles, settings, Allocator.TempJob, out this.report, this.levels );
        }

        public Solved( NativeArray<float3> positions, NativeArray<float3> normals, NativeArray<BurstMesh.Triangle> triangles, in BurstFields.Settings settings ) {
            this.settings = settings;
            this.fields   = BurstFields.solve( positions, normals, triangles, settings, Allocator.TempJob, out this.report, this.levels );
        }

        public void Dispose( ) {
            this.fields.Dispose( );
            this.levels.Dispose( );
        }

        public BurstFields.Hierarchy hierarchy {
            get { return this.fields.hierarchy; }
        }

        public int vertexCount {
            get { return this.hierarchy.levelStarts[1]; }
        }

        public bool leftOut( int vertex ) {
            return this.hierarchy.linkStarts[vertex] == this.hierarchy.linkStarts[vertex + 1];
        }

        public IEnumerable<BurstFields.Link> linksOf( int vertex ) {
            for( var link = this.hierarchy.linkStarts[vertex]; link < this.hierarchy.linkStarts[vertex + 1]; ++link ) {
                yield return this.hierarchy.links[link];
            }
        }

        public IEnumerable<int> verticesOf( int level ) {
            return Enumerable.Range( this.hierarchy.levelStarts[level], this.hierarchy.levelStarts[level + 1] - this.hierarchy.levelStarts[level] );
        }

        public BurstFields.Energy energyOf( int level ) {
            using var energy = new NativeArray<BurstFields.Energy>( 1, Allocator.TempJob );

            new BurstFields.EnergyJob {
                view = new BurstFields.View( this.hierarchy ), level = level, positions = true, rho = ( float )this.settings.length, symmetry = this.settings.symmetry, energy = energy
            }.Run( );

            return energy[0];
        }

        public BurstFields.Singularities singularities( ) {
            using var singularities = new NativeArray<BurstFields.Singularities>( 1, Allocator.TempJob );

            new BurstFields.SingularitiesJob {
                view          = new BurstFields.View( this.hierarchy ),
                rho           = ( float )this.settings.length,
                symmetry      = this.settings.symmetry,
                triangles     = this.fields.mesh.mesh.triangles.AsArray( ),
                subMeshes     = this.fields.mesh.mesh.subMeshes.AsArray( ),
                singularities = singularities
            }.Run( );

            return singularities[0];
        }

    }

    [Test]
    public void theLongestEdgeIsHalfTheLengthOrTwiceTheMeanEdge( ) {
        Assert.AreEqual( 0.5, BurstFields.longestEdge( 1.0, 10.0 ), "half the length" );
        Assert.AreEqual( 2.0, BurstFields.longestEdge( 8.0, 1.0 ),  "twice the mean edge" );
    }

    [Test]
    public void everyEdgeOfTheMeshTheFieldsAreOnIsShortEnough( [Values( 1.0, 2.5 )] double length ) {
        var size  = new Vector3Int( 3, 2, 1 );
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, size, ( centre, axis, side ) => axis );

        using var solved = new Solved( built, settingsOf( Symmetry.Quads, length ) );

        var mesh  = solved.fields.mesh.mesh;
        var mean  = Enumerable.Range( 0, built.triangles.Count ).SelectMany( ( t ) => edgesOf( built.triangles[t] ) ).Distinct( ).Average( ( edge ) => Vector3.Distance( built.positions[edge.a], built.positions[edge.b] ) );
        var limit = BurstFields.longestEdge( length, mean );

        Assert.AreEqual( limit, math.min( length * 0.5, mean * 2.0 ), 1e-12, "the limit" );

        var longest = 0.0;

        for( var edge = 0; edge < mesh.edgeCount; ++edge ) {
            longest = math.max( longest, math.distance( mesh.positions[mesh.edges[edge].x], mesh.positions[mesh.edges[edge].y] ) );
        }

        Assert.LessOrEqual( longest, limit + 1e-12, "the longest edge" );
        Assert.Greater( longest, limit * 0.5, "and edges are not split further than that needs" );

        // the same surface: every vertex on the box, and every sub-mesh with the area it had
        var areas = new double[3];

        for( var triangle = 0; triangle < mesh.triangleCount; ++triangle ) {
            var corners = mesh.triangles[triangle];

            areas[mesh.subMeshes[triangle]] += 0.5 * math.length( math.cross( mesh.positions[corners.y] - mesh.positions[corners.x], mesh.positions[corners.z] - mesh.positions[corners.x] ) );
        }

        Assert.AreEqual( 2.0 * size.y * size.z, areas[0], 1e-9, "the faces along x" );
        Assert.AreEqual( 2.0 * size.z * size.x, areas[1], 1e-9, "the faces along y" );
        Assert.AreEqual( 2.0 * size.x * size.y, areas[2], 1e-9, "the faces along z" );

        for( var vertex = 0; vertex < mesh.vertexCount; ++vertex ) {
            var position = mesh.positions[vertex];

            Assert.IsTrue( math.all( position >= 0.0 ) && math.all( position <= new double3( size.x, size.y, size.z ) ), $"vertex {vertex} is within the box" );
            Assert.IsTrue( Enumerable.Range( 0, 3 ).Any( ( axis ) => position[axis] == 0.0 || position[axis] == size[axis] ), $"vertex {vertex} is on a face of the box" );
        }

        Assert.AreEqual( mesh.vertexCount - built.positions.Count,                   solved.report.splits,    "splits" );
        Assert.AreEqual( built.triangles.Count + ( 2 * solved.report.splits ),       solved.report.triangles, "two triangles more a split" );
        Assert.AreEqual( mesh.vertexCount,                                           solved.report.vertices,  "vertices" );
        Assert.AreEqual( mesh.vertexCount,                                           solved.vertexCount,      "a vertex of level 0 for every vertex of the mesh" );
    }

    private static IEnumerable<( int a, int b )> edgesOf( BurstMesh.Triangle triangle ) {
        yield return ( math.min( triangle.a, triangle.b ), math.max( triangle.a, triangle.b ) );
        yield return ( math.min( triangle.b, triangle.c ), math.max( triangle.b, triangle.c ) );
        yield return ( math.min( triangle.c, triangle.a ), math.max( triangle.c, triangle.a ) );
    }

    [Test]
    public void theFeaturesOfTheMeshTheFieldsAreOnAreFoundAtTheFeatureLength( ) {
        // a strip 0.42 wide that bridges a right angle between two faces, along x: two creases by each edge's own angle,
        // and at a length of 1 one, which is the strip merged into a face (BurstFeatures)
        var built = new HandBuiltMesh( );

        for( var x = 0; x < 6; ++x ) {
            for( var row = 0; row < 6; ++row ) {
                var from = row == 0 ? 0.3f : row;

                built.quad( new Vector3( x, from, 0 ), new Vector3( x + 1, from, 0 ), new Vector3( x + 1, row + 1, 0 ), new Vector3( x, row + 1, 0 ) );
                built.quad( new Vector3( x, 0, from ), new Vector3( x, 0, row + 1 ), new Vector3( x + 1, 0, row + 1 ), new Vector3( x + 1, 0, from ) );
            }

            built.quad( new Vector3( x, 0, 0.3f ), new Vector3( x + 1, 0, 0.3f ), new Vector3( x + 1, 0.3f, 0 ), new Vector3( x, 0.3f, 0 ) );
        }

        using var positions = built.nativePositions( );
        using var normals   = built.nativeNormals( );
        using var triangles = built.nativeTriangles( );

        foreach( var ( featureLength, expected ) in new[] { ( 0.0, 2 ), ( 1.0, 1 ) } ) {
            var settings = settingsOf( Symmetry.Quads, 4.0 );

            settings.featureLength = featureLength;

            var report = new BurstFields.Report( );

            using var fields = BurstFields.prepare( positions, normals, triangles, settings, Allocator.TempJob, ref report );

            var lines = new HashSet<int>( );

            for( var edge = 0; edge < fields.mesh.mesh.edgeCount; ++edge ) {
                if( fields.mesh.mesh.edges[edge].x != -1 && fields.mesh.edgeLines[edge] != -1 && fields.mesh.mesh.edgeSharp[edge] == BurstMeshSimplifier.Sharp.None ) {
                    lines.Add( fields.mesh.edgeLines[edge] );
                }
            }

            Assert.AreEqual( expected, lines.Count, $"lines by angle at a feature length of {featureLength}" );
        }
    }

    [Test]
    public void levelZeroOfAFlatPatchHasItsNormalsItsAreasAndItsRimHeld( ) {
        // at a length of 4 no edge is split: the longest is a diagonal
        var built = new HandBuiltMesh( ).patch( 4, 4 );

        using var solved = new Solved( built, settingsOf( Symmetry.Quads, 4.0 ) );

        var hierarchy = solved.hierarchy;

        Assert.AreEqual( 0,  solved.report.splits,  "splits" );
        Assert.AreEqual( 25, solved.vertexCount,    "vertices" );
        Assert.AreEqual( 0,  solved.report.leftOut, "left out" );
        Assert.AreEqual( 16, solved.report.held,    "held: the rim" );

        var total = 0.0;

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            var position = built.positions[vertex];
            var onRim    = position.x == 0.0f || position.x == 4.0f || position.y == 0.0f || position.y == 4.0f;
            var corner   = ( position.x == 0.0f || position.x == 4.0f ) && ( position.y == 0.0f || position.y == 4.0f );

            Assert.AreEqual( ( float3 )position,            hierarchy.positions[vertex], $"vertex {vertex}: where it is" );
            Assert.AreEqual( new float3( 0.0f, 0.0f, 1.0f ), hierarchy.normals[vertex],  $"vertex {vertex}: its normal" );
            Assert.AreEqual( onRim ? 1 : 0,                  hierarchy.held[vertex],     $"vertex {vertex} at {position}: held" );

            total += hierarchy.areas[vertex];

            if( !onRim ) {
                // a square of half a cell each way: its triangles' circumcentres are the middles of the diagonals
                Assert.AreEqual( 1.0f, hierarchy.areas[vertex], 1e-6f, $"vertex {vertex}: its Voronoi area" );

                Assert.AreEqual( 6, solved.linksOf( vertex ).Count( ), $"vertex {vertex}: neighbours" );
            }
            else {
                var along = hierarchy.heldDirections[vertex];

                // half of that on a rim, and a quarter at a corner, whichever way its square is cut
                Assert.AreEqual( corner ? 0.25f : 0.5f, hierarchy.areas[vertex], 1e-6f, $"vertex {vertex} at {position}: its Voronoi area" );

                Assert.AreEqual( ( float3 )position, hierarchy.heldPositions[vertex], $"vertex {vertex}: held at itself" );
                Assert.AreEqual( 1.0f, math.length( along ), 1e-6f, $"vertex {vertex}: a direction of unit length" );

                if( !corner ) {
                    var alongX = position.y == 0.0f || position.y == 4.0f;

                    Assert.AreEqual( 1.0f, math.abs( alongX ? along.x : along.y ), 1e-6f, $"vertex {vertex} at {position}: held along its rim" );
                }
                else {
                    Assert.AreEqual( 1.0f, math.max( math.abs( along.x ), math.abs( along.y ) ), 1e-6f, $"vertex {vertex} at {position}: held along one of its two rims" );
                }
            }

            // neighbours in the order of their vertices, each once, each at a weight of one, and each way round
            var links = solved.linksOf( vertex ).ToArray( );

            CollectionAssert.AreEqual( links.Select( ( link ) => link.vertex ).OrderBy( ( other ) => other ).Distinct( ).ToArray( ), links.Select( ( link ) => link.vertex ).ToArray( ), $"vertex {vertex}: links in order" );

            foreach( var link in links ) {
                Assert.AreEqual( 1.0f, link.weight, $"vertex {vertex}: weight" );
                Assert.IsTrue( solved.linksOf( link.vertex ).Any( ( back ) => back.vertex == vertex ), $"vertex {vertex}: its neighbour {link.vertex} has it too" );
                Assert.IsTrue( edgesOf( built ).Contains( ( math.min( vertex, link.vertex ), math.max( vertex, link.vertex ) ) ), $"vertex {vertex}: {link.vertex} is across an edge" );
            }
        }

        Assert.AreEqual( 16.0, total, 1e-5, "the areas are the patch's" );
        Assert.AreEqual( edgesOf( built ).Count * 2, hierarchy.linkStarts[solved.vertexCount], "two links an edge" );
    }

    private static HashSet<( int, int )> edgesOf( HandBuiltMesh built ) {
        return new HashSet<( int, int )>( built.triangles.SelectMany( ( triangle ) => edgesOf( triangle ) ) );
    }

    [Test]
    public void aVertexOfACreaseTakesTheNormalOfOneOfItsTriangles( [Values] bool creases ) {
        // at a length of 4 the box of 2 keeps its 26 vertices
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 2 );

        using var solved = new Solved( built, settingsOf( Symmetry.Quads, 4.0, creases ) );

        Assert.AreEqual( 0, solved.report.splits, "splits" );
        Assert.AreEqual( 0, solved.report.held,   "held" );

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            var position = built.positions[vertex];
            var normal   = solved.hierarchy.normals[vertex];
            var outward  = new float3( position.x == 0.0f ? -1 : position.x == 2.0f ? 1 : 0, position.y == 0.0f ? -1 : position.y == 2.0f ? 1 : 0, position.z == 0.0f ? -1 : position.z == 2.0f ? 1 : 0 );
            var faces    = ( int )math.csum( math.abs( outward ) );

            Assert.AreEqual( 1.0f, math.length( normal ), 1e-6f, $"vertex {vertex}: unit length" );

            if( faces == 1 ) {
                Assert.Less( math.distance( outward, normal ), 1e-6f, $"vertex {vertex} at {position}: within a face" );
            }
            else if( creases ) {
                // the normal of a face it is on
                Assert.AreEqual( 1.0f, math.cmax( normal * outward ), 1e-6f, $"vertex {vertex} at {position}: the normal {normal} of one of its faces" );
                Assert.AreEqual( 1.0f, math.csum( math.abs( normal ) ), 1e-6f, $"vertex {vertex} at {position}: along an axis" );
            }
            else if( faces == 3 ) {
                // three right angles: the mean of the three faces
                Assert.Less( math.distance( math.normalize( outward ), normal ), 1e-6f, $"vertex {vertex} at {position}: between its three faces" );
            }
            else {
                // on an edge, between its two faces, which have a straight angle each at it
                Assert.Less( math.distance( math.normalize( outward ), normal ), 1e-6f, $"vertex {vertex} at {position}: between its two faces" );
            }
        }
    }

    // level 0 of a mesh as it is, without the sheets or the splits
    private static void levelZero( HandBuiltMesh built, Action<BurstFields.Hierarchy, BurstFields.Report> then, bool freeMaterials = false ) {
        using var positions = built.nativePositions( );
        using var normals   = built.nativeNormals( );
        using var triangles = built.nativeTriangles( );
        using var lineCount = new NativeArray<int>( 1, Allocator.TempJob );
        using var mesh      = new BurstIsotropic.FeatureMesh( positions.Length, triangles.Length, Allocator.TempJob );
        using var hierarchy = new BurstFields.Hierarchy( Allocator.TempJob );

        new BurstMeshSimplifier.BuildJob { positions = positions, normals = normals, triangles = triangles, mesh = mesh.mesh }.Run( );

        new BurstFeatures.SkeletonJob { cosine = math.cos( math.radians( featureAngle ) ), mesh = mesh.mesh, edgeLines = mesh.edgeLines, corners = mesh.corners, lineCount = lineCount }.Run( );

        var report = new BurstFields.Report( );

        BurstFields.graph( mesh, hierarchy, true, ref report, freeMaterials );

        then( hierarchy, report );
    }

    [Test]
    public void aVertexOfAnEdgeOfFourTrianglesIsLeftOutOfTheFields( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 2 ).box( new Vector3Int( 2, 2, 0 ), Vector3Int.one * 2 );

        levelZero( built, ( hierarchy, report ) => {
            // the three vertices of the edge the boxes share
            Assert.AreEqual( 3, report.leftOut, "left out" );

            for( var vertex = 0; vertex < built.positions.Count; ++vertex ) {
                var position = built.positions[vertex];
                var shared   = position.x == 2.0f && position.y == 2.0f;
                var links    = hierarchy.linkStarts[vertex + 1] - hierarchy.linkStarts[vertex];

                Assert.AreEqual( shared, links == 0, $"vertex {vertex} at {position}: without neighbours" );

                for( var link = hierarchy.linkStarts[vertex]; link < hierarchy.linkStarts[vertex + 1]; ++link ) {
                    var other = built.positions[hierarchy.links[link].vertex];

                    Assert.IsFalse( other.x == 2.0f && other.y == 2.0f, $"vertex {vertex} at {position}: no neighbour on the edge" );
                }
            }
        } );
    }

    [Test]
    public void aVertexOfALineBetweenMaterialsIsHeldAlongIt( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 4, 2, 2 ), ( centre, axis, side ) => centre.x < 2.0f ? 0 : 1 );

        levelZero( built, ( hierarchy, report ) => {
            // the line goes round the box at x = 2: eight vertices
            Assert.AreEqual( 8, report.held, "held" );

            for( var vertex = 0; vertex < built.positions.Count; ++vertex ) {
                var position = built.positions[vertex];

                Assert.AreEqual( position.x == 2.0f ? 1 : 0, hierarchy.held[vertex], $"vertex {vertex} at {position}: held" );

                if( position.x == 2.0f ) {
                    var along = hierarchy.heldDirections[vertex];

                    // at an edge of the box the line goes over it, and the vertex has the normal of one of the two
                    // faces: the edge of the line in that face is the one it is held along
                    Assert.AreEqual( 0.0f, along.x, 1e-6f, $"vertex {vertex} at {position}: along the line, which is across x" );
                    Assert.AreEqual( 1.0f, math.length( along ), 1e-6f, $"vertex {vertex}: unit length" );
                    Assert.AreEqual( 0.0f, math.dot( along, hierarchy.normals[vertex] ), 1e-6f, $"vertex {vertex}: in its tangent plane" );
                    Assert.AreEqual( 1.0f, math.cmax( math.abs( along ) ), 1e-6f, $"vertex {vertex} at {position}: along an edge of the line" );
                }
            }
        } );
    }

    // with lines between materials left free, which is for measuring what holding them does, only open edges hold
    [Test]
    public void withMaterialsFreeOnlyOpenEdgesHold( ) {
        var box = new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 4, 2, 2 ), ( centre, axis, side ) => centre.x < 2.0f ? 0 : 1 );

        levelZero( box, ( hierarchy, report ) => Assert.AreEqual( 0, report.held, "held on a closed box" ), freeMaterials: true );

        var patch = new HandBuiltMesh( ).patch( 4, 4, ( i, j ) => i < 2 ? 0 : 1 );

        levelZero( patch, ( hierarchy, report ) => Assert.AreEqual( 16, report.held, "held on an open patch: its rim alone" ), freeMaterials: true );
        levelZero( patch, ( hierarchy, report ) => Assert.AreEqual( 16 + 3, report.held, "held on an open patch: its rim and the line inside it" ) );
    }

    // ---- the hierarchy and its colours ----

    private static HandBuiltMesh twoBoxes( ) {
        return new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 4, 3, 2 ) ).box( new Vector3Int( 6, 0, 0 ), Vector3Int.one * 2 );
    }

    [Test]
    public void everyLevelIsAboutHalfTheOneBelowUntilEveryPieceIsOneVertex( [Values] Symmetry symmetry ) {
        using var solved = new Solved( twoBoxes( ), settingsOf( symmetry, 0.5 ) );

        var hierarchy = solved.hierarchy;

        Assert.Greater( solved.vertexCount, 2000, "vertices" );
        Assert.AreEqual( solved.report.levels, hierarchy.levelCount, "levels" );
        Assert.LessOrEqual( hierarchy.levelCount, 1.5 * math.log2( solved.vertexCount ), "levels: about one for every halving" );

        for( var level = 0; level + 1 < hierarchy.levelCount; ++level ) {
            var below = hierarchy.levelStarts[level + 1] - hierarchy.levelStarts[level];
            var above = hierarchy.levelStarts[level + 2] - hierarchy.levelStarts[level + 1];

            Assert.Less( above, below, $"level {level + 1} is smaller than level {level}" );
            Assert.GreaterOrEqual( above * 2, below, $"level {level + 1} is at least half of level {level}" );

            if( below >= 64 ) {
                Assert.LessOrEqual( above, below * 0.75, $"level {level + 1}, of {above}, is about half of level {level}, of {below}" );
            }
        }

        // the two boxes
        Assert.AreEqual( 2, solved.verticesOf( hierarchy.levelCount - 1 ).Count( ), "vertices at the top" );
        Assert.AreEqual( hierarchy.linkStarts[hierarchy.levelStarts[hierarchy.levelCount - 1]], hierarchy.links.Length, "no links at the top" );
    }

    [Test]
    public void scoredByTheSmallerRatioTheLevelsStopGettingSmaller( ) {
        // finding 22: the paper's score, which is here to be measured against
        using var solved = new Solved( twoBoxes( ), settingsOf( Symmetry.Quads, 0.5, smallerRatio: true ) );

        Assert.AreEqual( BurstFields.mostLevels, solved.hierarchy.levelCount, "levels" );
        Assert.Greater( solved.verticesOf( solved.hierarchy.levelCount - 1 ).Count( ), 100, "vertices at the top" );
    }

    [Test]
    public void aMergedVertexIsTheSumOfItsTwo( ) {
        using var solved = new Solved( twoBoxes( ), settingsOf( Symmetry.Quads, 0.5 ) );

        var hierarchy = solved.hierarchy;
        var total     = solved.verticesOf( 0 ).Sum( ( vertex ) => ( double )hierarchy.areas[vertex] );

        Assert.AreEqual( 2.0 * ( ( 4 * 3 ) + ( 3 * 2 ) + ( 2 * 4 ) ) + 24.0, total, 1e-3, "the areas of level 0 are the two boxes'" );

        for( var level = 1; level < hierarchy.levelCount; ++level ) {
            Assert.AreEqual( total, solved.verticesOf( level ).Sum( ( vertex ) => ( double )hierarchy.areas[vertex] ), 1e-3, $"the areas of level {level}" );

            var seen = new HashSet<int>( );

            foreach( var vertex in solved.verticesOf( level ) ) {
                var below = hierarchy.children[vertex];

                Assert.IsTrue( below.x >= hierarchy.levelStarts[level - 1] && below.x < hierarchy.levelStarts[level], $"vertex {vertex}: its first is of the level below" );
                Assert.AreEqual( vertex, hierarchy.parents[below.x], $"vertex {vertex}: its first knows it" );
                Assert.IsTrue( seen.Add( below.x ), $"vertex {below.x} is merged once" );

                if( below.y == -1 ) {
                    Assert.AreEqual( hierarchy.positions[below.x], hierarchy.positions[vertex], $"vertex {vertex}: where its one is" );
                    Assert.AreEqual( hierarchy.normals[below.x],   hierarchy.normals[vertex],   $"vertex {vertex}: its one's normal" );
                    Assert.AreEqual( hierarchy.areas[below.x],     hierarchy.areas[vertex],     $"vertex {vertex}: its one's area" );

                    continue;
                }

                Assert.IsTrue( below.y >= hierarchy.levelStarts[level - 1] && below.y < hierarchy.levelStarts[level], $"vertex {vertex}: its second is of the level below" );
                Assert.AreEqual( vertex, hierarchy.parents[below.y], $"vertex {vertex}: its second knows it" );
                Assert.IsTrue( seen.Add( below.y ), $"vertex {below.y} is merged once" );
                Assert.IsTrue( solved.linksOf( below.x ).Any( ( link ) => link.vertex == below.y ), $"vertex {vertex}: its two were neighbours" );

                var a = ( double )hierarchy.areas[below.x];
                var b = ( double )hierarchy.areas[below.y];

                Assert.AreEqual( a + b, hierarchy.areas[vertex], 1e-5 * ( a + b ), $"vertex {vertex}: the sum of the areas" );

                Assert.Less(
                    math.distance( ( ( d( hierarchy.positions[below.x] ) * a ) + ( d( hierarchy.positions[below.y] ) * b ) ) / ( a + b ), d( hierarchy.positions[vertex] ) ), 1e-5,
                    $"vertex {vertex}: between its two, by their areas"
                );

                var normal = ( d( hierarchy.normals[below.x] ) * a ) + ( d( hierarchy.normals[below.y] ) * b );

                if( math.length( normal ) > 1e-3 ) {
                    Assert.Less( math.distance( math.normalize( normal ), d( hierarchy.normals[vertex] ) ), 1e-4, $"vertex {vertex}: its normal between its two's, by their areas" );
                }
            }

            Assert.AreEqual( solved.verticesOf( level - 1 ).Count( ), seen.Count, $"every vertex of level {level - 1} is merged into one of level {level}" );
        }

        foreach( var vertex in solved.verticesOf( hierarchy.levelCount - 1 ) ) {
            Assert.AreEqual( -1, hierarchy.parents[vertex], $"vertex {vertex} of the top level has none above it" );
        }
    }

    [Test]
    public void theNeighboursOfAMergedVertexAreThoseOfItsTwo( ) {
        using var solved = new Solved( twoBoxes( ), settingsOf( Symmetry.Quads, 0.5 ) );

        var hierarchy = solved.hierarchy;

        for( var level = 1; level < hierarchy.levelCount; ++level ) {
            foreach( var vertex in solved.verticesOf( level ) ) {
                var expected = new SortedDictionary<int, float>( );

                for( var child = 0; child < 2; ++child ) {
                    var below = hierarchy.children[vertex][child];

                    if( below == -1 ) {
                        continue;
                    }

                    foreach( var link in solved.linksOf( below ) ) {
                        var above = hierarchy.parents[link.vertex];

                        if( above != vertex ) {
                            expected[above] = ( expected.TryGetValue( above, out var weight ) ? weight : 0.0f ) + link.weight;
                        }
                    }
                }

                var links = solved.linksOf( vertex ).ToArray( );

                CollectionAssert.AreEqual( expected.Keys.ToArray( ), links.Select( ( link ) => link.vertex ).ToArray( ), $"vertex {vertex} of level {level}: neighbours, in order" );
                CollectionAssert.AreEqual( expected.Values.ToArray( ), links.Select( ( link ) => link.weight ).ToArray( ), $"vertex {vertex} of level {level}: weights" );
            }
        }
    }

    [Test]
    public void noTwoNeighboursAreOfOneColour( ) {
        using var solved = new Solved( twoBoxes( ), settingsOf( Symmetry.Quads, 0.5 ) );

        var hierarchy = solved.hierarchy;
        var colours   = new int[hierarchy.vertexCount];
        var seen      = new bool[hierarchy.vertexCount];

        Assert.AreEqual( hierarchy.vertexCount, hierarchy.order.Length, "a place in the order for every vertex" );
        Assert.AreEqual( hierarchy.levelCount + 1, hierarchy.levelColours.Length, "colours for every level" );
        Assert.AreEqual( hierarchy.vertexCount, hierarchy.colourStarts[hierarchy.colourStarts.Length - 1], "the colours end where the vertices do" );

        for( var level = 0; level < hierarchy.levelCount; ++level ) {
            Assert.AreEqual( hierarchy.levelStarts[level], hierarchy.colourStarts[hierarchy.levelColours[level]], $"the colours of level {level} start where it does" );

            for( var colour = hierarchy.levelColours[level]; colour < hierarchy.levelColours[level + 1]; ++colour ) {
                Assert.Less( hierarchy.colourStarts[colour], hierarchy.colourStarts[colour + 1], $"colour {colour} of level {level} has vertices" );

                for( var index = hierarchy.colourStarts[colour]; index < hierarchy.colourStarts[colour + 1]; ++index ) {
                    var vertex = hierarchy.order[index];

                    Assert.IsTrue( vertex >= hierarchy.levelStarts[level] && vertex < hierarchy.levelStarts[level + 1], $"vertex {vertex} is of level {level}" );
                    Assert.IsFalse( seen[vertex], $"vertex {vertex} is in the order once" );

                    seen[vertex]    = true;
                    colours[vertex] = colour;
                }
            }
        }

        Assert.IsTrue( seen.All( ( once ) => once ), "every vertex is in the order" );

        for( var vertex = 0; vertex < hierarchy.vertexCount; ++vertex ) {
            foreach( var link in solved.linksOf( vertex ) ) {
                Assert.AreNotEqual( colours[vertex], colours[link.vertex], $"vertex {vertex} and its neighbour {link.vertex}" );
            }
        }

        Assert.AreEqual( hierarchy.levelColours[1] - hierarchy.levelColours[0], solved.report.colours, "the colours of level 0" );
        Assert.LessOrEqual( solved.report.colours, 8, "the colours of level 0" );
    }

    // ---- the fields ----

    private static void assertTheSameFields( Solved first, Solved second, string message ) {
        CollectionAssert.AreEqual( first.hierarchy.orientations.AsArray( ).ToArray( ), second.hierarchy.orientations.AsArray( ).ToArray( ), $"{message}: directions" );
        CollectionAssert.AreEqual( first.hierarchy.lattice.AsArray( ).ToArray( ),      second.hierarchy.lattice.AsArray( ).ToArray( ),      $"{message}: lattice points" );
    }

    // what holds of the fields of any mesh: at every vertex that is not left out a direction of unit length in its
    // tangent plane, and a lattice point in that plane no further from the vertex than the middle of a cell is
    private static void assertFieldsAreWellFormed( Solved solved ) {
        var hierarchy = solved.hierarchy;
        var rho       = solved.settings.length;
        var furthest  = solved.settings.symmetry == Symmetry.Quads ? rho * math.SQRT2 / 2.0 : rho / math.sqrt( 3.0 );

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            if( solved.leftOut( vertex ) ) {
                continue;
            }

            var normal    = d( hierarchy.normals[vertex] );
            var direction = d( hierarchy.orientations[vertex] );
            var point     = d( hierarchy.lattice[vertex] );
            var position  = d( hierarchy.positions[vertex] );

            Assert.AreEqual( 1.0, math.length( direction ), 1e-4, $"vertex {vertex}: a direction of unit length" );
            Assert.AreEqual( 0.0, math.dot( normal, direction ), 1e-4, $"vertex {vertex}: a direction in its tangent plane" );
            Assert.AreEqual( 0.0, math.dot( normal, point - position ), 1e-4 * rho, $"vertex {vertex}: a lattice point in its tangent plane" );
            Assert.LessOrEqual( math.distance( point, position ), furthest * 1.001, $"vertex {vertex}: its lattice point is the nearest to it" );
        }
    }

    [Test]
    public void aFlatSquareHasFieldsAlongItsSidesAndASquareLatticeBetweenThem( ) {
        // eight cells a side at a length of 2: the lattice fits between the rims, which hold it
        var built = new HandBuiltMesh( ).patch( 8, 8 );

        using var solved = new Solved( built, settingsOf( Symmetry.Quads, 2.0, measure: true ) );

        var hierarchy = solved.hierarchy;

        assertFieldsAreWellFormed( solved );

        Assert.Greater( solved.report.splits, 0, "splits: the diagonals" );

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            var direction = hierarchy.orientations[vertex];
            var point     = hierarchy.lattice[vertex];

            Assert.AreEqual( 1.0f, math.max( math.abs( direction.x ), math.abs( direction.y ) ), 1e-5f, $"vertex {vertex}: a direction along a side" );

            // a point of the lattice of twos
            Assert.AreEqual( 0.0f, math.abs( ( point.x / 2.0f ) - math.round( point.x / 2.0f ) ), 1e-3f, $"vertex {vertex}: its lattice point {point} across" );
            Assert.AreEqual( 0.0f, math.abs( ( point.y / 2.0f ) - math.round( point.y / 2.0f ) ), 1e-3f, $"vertex {vertex}: its lattice point {point} up" );
        }

        var energy = solved.energyOf( 0 );

        Assert.AreEqual( 0.0, energy.orientation, 1e-6, "the directions of neighbours are the same" );
        Assert.AreEqual( 0.0, energy.position,    1e-6, "the lattices of neighbours are the same" );

        Assert.AreEqual( 0, solved.report.singularities.orientation, "irregular points of the directions" );
        Assert.AreEqual( 0, solved.report.singularities.position,    "irregular points of the lattice" );
    }

    // a flat triangle of equal sides, of so many small ones a side, its rim open
    private static HandBuiltMesh flatTriangle( int size ) {
        var built = new HandBuiltMesh( );

        // rows of points, each half a step along from the one below
        Vector3 at( int i, int j ) {
            return new Vector3( i + ( 0.5f * j ), ( float )( j * Math.Sqrt( 3.0 ) / 2.0 ), 0.0f );
        }

        for( var j = 0; j < size; ++j )
        for( var i = 0; i + j < size; ++i ) {
            built.triangle( at( i, j ), at( i + 1, j ), at( i, j + 1 ) );

            if( i + j + 1 < size ) {
                built.triangle( at( i + 1, j ), at( i + 1, j + 1 ), at( i, j + 1 ) );
            }
        }

        return built;
    }

    [Test]
    public void aFlatTriangleHasFieldsAlongItsSidesAndATriangularLatticeBetweenThem( ) {
        // nine edges a side at a length of 3: the lattice fits between the rims, which hold it, and no edge is split
        var built = flatTriangle( 9 );

        using var solved = new Solved( built, settingsOf( Symmetry.Triangles, 3.0, measure: true ) );

        var hierarchy = solved.hierarchy;

        assertFieldsAreWellFormed( solved );

        Assert.AreEqual( 0, solved.report.splits, "splits" );

        var second = new double3( 0.5, math.sqrt( 3.0 ) / 2.0, 0.0 );

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            var direction = d( hierarchy.orientations[vertex] );
            var point     = d( hierarchy.lattice[vertex] );

            // along a side: a sixth turn apart
            var angle = math.degrees( math.atan2( direction.y, direction.x ) ) / 60.0;

            Assert.AreEqual( 0.0, math.abs( angle - math.round( angle ) ), 1e-3, $"vertex {vertex}: a direction along a side" );

            // a point of the lattice of threes along the first side and the second
            var along = point.y / second.y / 3.0;
            var first = ( point.x - ( along * 3.0 * second.x ) ) / 3.0;

            Assert.AreEqual( 0.0, math.abs( along - math.round( along ) ), 1e-3, $"vertex {vertex}: its lattice point {point} along the second side" );
            Assert.AreEqual( 0.0, math.abs( first - math.round( first ) ), 1e-3, $"vertex {vertex}: its lattice point {point} along the first side" );
        }

        var energy = solved.energyOf( 0 );

        Assert.AreEqual( 0.0, energy.orientation, 1e-4, "the directions of neighbours are the same" );
        Assert.AreEqual( 0.0, energy.position,    1e-6, "the lattices of neighbours are the same" );

        Assert.AreEqual( 0, solved.report.singularities.orientation, "irregular points of the directions" );
        Assert.AreEqual( 0, solved.report.singularities.position,    "irregular points of the lattice" );
    }

    [Test]
    public void aBoxHasDirectionsAlongItsEdgesAndAnIrregularPointAtEachCorner( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one * 4 );

        using var solved = new Solved( built, settingsOf( Symmetry.Quads, 1.0, measure: true ) );

        var hierarchy = solved.hierarchy;

        assertFieldsAreWellFormed( solved );

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            Assert.AreEqual( 1.0f, math.cmax( math.abs( hierarchy.orientations[vertex] ) ), 1e-4f, $"vertex {vertex} at {hierarchy.positions[vertex]}: a direction along an edge of the box" );
        }

        // three faces meet at a corner, a quarter turn short of four
        Assert.AreEqual( 8, solved.report.singularities.orientation, "irregular points of the directions" );
        Assert.AreEqual( 0, solved.report.singularities.position,    "irregular points of the lattice" );

        // the lattice of every face lies on the box's edges: its points are whole lengths from them
        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            var point = hierarchy.lattice[vertex];

            // measured: 0.016 at the most, at the corners, which the smoothing only nears
            Assert.AreEqual( 0.0f, math.cmax( math.abs( point - math.round( point ) ) ), 0.05f, $"vertex {vertex} at {hierarchy.positions[vertex]}: its lattice point {point}" );
        }

        Assert.AreEqual( 0.0, solved.energyOf( 0 ).position, 1e-4, "the lattices of neighbours are the same" );
    }

    [Test]
    public void aHeldVertexHasItsDirectionAlongItsLineAndItsLatticePointOnIt( [Values] Symmetry symmetry ) {
        // a band of a second material across a patch, whose lines are not a whole number of lengths from the rims
        var built = new HandBuiltMesh( ).patch( 8, 8, ( i, j ) => i >= 3 && i < 5 ? 1 : 0 );

        using var solved = new Solved( built, settingsOf( symmetry, 2.0 ) );

        var hierarchy = solved.hierarchy;
        var steps     = ( int )symmetry;
        var held      = 0;

        assertFieldsAreWellFormed( solved );

        for( var vertex = 0; vertex < solved.vertexCount; ++vertex ) {
            var position = hierarchy.positions[vertex];
            var onLine   = position.x == 0.0f || position.x == 8.0f || position.y == 0.0f || position.y == 8.0f || position.x == 3.0f || position.x == 5.0f;

            Assert.AreEqual( onLine ? 1 : 0, hierarchy.held[vertex], $"vertex {vertex} at {position}: held" );

            if( !onLine ) {
                continue;
            }

            ++held;

            var along     = d( hierarchy.heldDirections[vertex] );
            var direction = d( hierarchy.orientations[vertex] );
            var point     = d( hierarchy.lattice[vertex] ) - d( position );

            // the direction is the line's at one of its turns
            var turn = math.degrees( math.acos( math.clamp( math.abs( math.dot( along, direction ) ), 0.0, 1.0 ) ) ) / ( 360.0 / steps );

            Assert.AreEqual( 0.0, math.abs( turn - math.round( turn ) ), 1e-3, $"vertex {vertex} at {position}: its direction {direction} against its line's {along}" );

            // the lattice point is on the line through the vertex, no further along it than half a length
            Assert.AreEqual( 0.0, math.length( point - ( along * math.dot( along, point ) ) ), 1e-4, $"vertex {vertex} at {position}: its lattice point is on its line" );
            Assert.LessOrEqual( math.abs( math.dot( along, point ) ), 1.0 + 1e-4, $"vertex {vertex} at {position}: and the nearest on it" );
        }

        // the rim's 32 and the 7 of each line that are not on it
        Assert.AreEqual( 46, held,               "held: the rim and the two lines" );
        Assert.AreEqual( 46, solved.report.held, "held" );
    }

    [Test]
    public void theDirectionsOfAMergedVertexAreMadeFromItsTwo( [Values] Symmetry symmetry ) {
        using var solved = new Solved( twoBoxes( ), settingsOf( symmetry, 0.5 ) );

        var hierarchy = solved.hierarchy;
        var steps     = ( int )symmetry;

        for( var vertex = hierarchy.levelStarts[1]; vertex < hierarchy.vertexCount; ++vertex ) {
            var below  = hierarchy.children[vertex];
            var normal = d( hierarchy.normals[vertex] );
            var actual = d( hierarchy.orientations[vertex] );

            Assert.AreEqual( 1.0, math.length( actual ), 1e-4, $"vertex {vertex}: unit length" );
            Assert.AreEqual( 0.0, math.dot( normal, actual ), 1e-4, $"vertex {vertex}: in its tangent plane" );

            // the sum of its two at the turns that bring them nearest, in its tangent plane. On a box many turns are
            // as near as each other, and any of them will do
            var sums = new List<double3>( );

            if( below.y == -1 ) {
                sums.Add( d( hierarchy.orientations[below.x] ) );
            }
            else {
                var best = double.NegativeInfinity;

                for( var pass = 0; pass < 2; ++pass )
                for( var k = 0; k < steps; ++k )
                for( var l = 0; l < steps; ++l ) {
                    var a   = rotated( d( hierarchy.orientations[below.x] ), d( hierarchy.normals[below.x] ), k * 2.0 * math.PI / steps );
                    var b   = rotated( d( hierarchy.orientations[below.y] ), d( hierarchy.normals[below.y] ), l * 2.0 * math.PI / steps );
                    var dot = math.dot( a, b );

                    if( pass == 0 ) {
                        best = math.max( best, dot );
                    }
                    else if( dot > best - 1e-4 ) {
                        sums.Add( a + b );
                    }
                }
            }

            var nearest = double.PositiveInfinity;

            foreach( var sum in sums ) {
                var projected = sum - ( normal * math.dot( normal, sum ) );

                if( math.length( projected ) > 1e-3 ) {
                    nearest = math.min( nearest, math.distance( math.normalize( projected ), actual ) );
                }
                else {
                    nearest = 0.0;
                }
            }

            Assert.Less( nearest, 1e-3, $"vertex {vertex}" );
        }
    }

    [Test]
    public void aMergedVertexIsHeldToTheLineOfItsLargerHeldOne( ) {
        // a band of a second material across a flat patch: every line is straight, and every tangent plane the same
        using var solved = new Solved( new HandBuiltMesh( ).patch( 8, 8, ( i, j ) => i >= 3 && i < 5 ? 1 : 0 ), settingsOf( Symmetry.Quads, 2.0 ) );

        var hierarchy = solved.hierarchy;
        var both      = 0;

        for( var vertex = hierarchy.levelStarts[1]; vertex < hierarchy.vertexCount; ++vertex ) {
            var below  = hierarchy.children[vertex];
            var first  = hierarchy.held[below.x] != 0;
            var second = below.y != -1 && hierarchy.held[below.y] != 0;

            Assert.AreEqual( first || second ? 1 : 0, hierarchy.held[vertex], $"vertex {vertex}: held where either of its two is" );

            if( !first && !second ) {
                continue;
            }

            var from = first && ( !second || hierarchy.areas[below.x] >= hierarchy.areas[below.y] ) ? below.x : below.y;

            both += first && second ? 1 : 0;

            Assert.Less( math.distance( hierarchy.heldDirections[from], hierarchy.heldDirections[vertex] ), 1e-6f, $"vertex {vertex}: the direction of its larger held one" );
            Assert.Less( math.distance( hierarchy.heldPositions[from],  hierarchy.heldPositions[vertex] ),  1e-6f, $"vertex {vertex}: the point of its larger held one" );
        }

        Assert.Greater( both, 10, "merged vertices whose two are both held" );
    }

    [Test]
    public void aHeldLatticeIsMovedAcrossItsLineByTheLeastThatPutsALineOfItOnIt( [Values] Symmetry symmetry ) {
        // a line along x through the origin, in the plane z = 0, and lattices of a length of 2 with a line along x
        var normal  = new float3( 0.0f, 0.0f, 1.0f );
        var along   = new float3( 1.0f, 0.0f, 0.0f );
        var spacing = symmetry == Symmetry.Quads ? 2.0 : math.sqrt( 3.0 );

        foreach( var rows in new[] { -2.4, -1.6, -0.6, -0.4, 0.0, 0.3, 0.45, 0.55, 1.2, 3.7 } ) {
            var point = new float3( 0.7f, ( float )( rows * spacing ), 0.0f );
            var moved = BurstFields.ontoHeldLine( point, normal, along, new float3( 5.0f, 0.0f, 0.0f ), 2.0f, symmetry );
            var whole = math.round( rows );

            // the nearest of its lines along x is on the line: the point is a whole number of them from it, the number
            // it was nearest, and is moved neither along the line nor out of the plane
            Assert.AreEqual( whole * spacing, moved.y, 1e-5, $"{rows} rows from the line: across" );
            Assert.AreEqual( 0.7f,            moved.x, 1e-6, $"{rows} rows from the line: along" );
            Assert.AreEqual( 0.0f,            moved.z, 1e-6, $"{rows} rows from the line: out of the plane" );
        }
    }

    [Test]
    public void aFieldIsHandedDownToBothOfAVertexsTwoInTheirOwnTangentPlanes( [Values] bool positions ) {
        // without creases, so that the vertices of the boxes' edges have normals of their own, between their faces'
        using var solved = new Solved( twoBoxes( ), settingsOf( Symmetry.Quads, 0.5, creases: false ) );

        var hierarchy    = solved.hierarchy;
        var orientations = hierarchy.orientations;
        var lattice      = hierarchy.lattice;
        var random       = new System.Random( 61 );
        var start        = hierarchy.levelStarts[1];
        var count        = hierarchy.levelStarts[2] - start;

        // fields of level 1 set by hand, and those of level 0 set to nothing
        for( var vertex = start; vertex < start + count; ++vertex ) {
            orientations[vertex] = ( float3 )tangent( random, d( hierarchy.normals[vertex] ) );
            lattice[vertex]      = hierarchy.positions[vertex] + ( float3 )( direction( random ) * 0.3 );
        }

        for( var vertex = 0; vertex < start; ++vertex ) {
            orientations[vertex] = float3.zero;
            lattice[vertex]      = float3.zero;
        }

        new BurstFields.HandDownJob { view = new BurstFields.View( hierarchy ), start = start, positions = positions }.Schedule( count, 16 ).Complete( );

        var turned = 0;

        for( var vertex = 0; vertex < start; ++vertex ) {
            var above  = hierarchy.parents[vertex];
            var normal = d( hierarchy.normals[vertex] );

            if( positions ) {
                var point    = d( lattice[above] );
                var expected = point - ( normal * math.dot( normal, point - d( hierarchy.positions[vertex] ) ) );

                Assert.Less( math.distance( expected, d( lattice[vertex] ) ), 1e-5, $"vertex {vertex}: the lattice point of {above}, in its own tangent plane" );
                Assert.AreEqual( float3.zero, orientations[vertex], $"vertex {vertex}: its direction is left" );
            }
            else {
                var given    = d( orientations[above] );
                var expected = given - ( normal * math.dot( normal, given ) );

                turned += math.length( expected ) < 0.999 ? 1 : 0;

                Assert.Less( math.distance( math.normalize( expected ), d( orientations[vertex] ) ), 1e-5, $"vertex {vertex}: the direction of {above}, in its own tangent plane and of unit length" );
                Assert.AreEqual( float3.zero, lattice[vertex], $"vertex {vertex}: its lattice point is left" );
            }
        }

        if( !positions ) {
            Assert.Greater( turned, 10, "vertices whose tangent plane is not that of the vertex above" );
        }
    }

    [Test]
    public void theSameMeshHasTheSameFieldsEveryTime( [Values] Symmetry symmetry ) {
        var built = twoBoxes( );

        using var first    = new Solved( built, settingsOf( symmetry, 0.5 ) );
        using var second   = new Solved( built, settingsOf( symmetry, 0.5 ) );
        using var measured = new Solved( built, settingsOf( symmetry, 0.5, measure: true ) );

        assertFieldsAreWellFormed( first );
        assertTheSameFields( first, second, "twice" );

        // measured, every level is smoothed colour by colour by every worker, and unmeasured the small ones by one
        assertTheSameFields( first, measured, "measured" );

        Assert.AreEqual( first.hierarchy.levelCount, measured.levels.Length, "a line for every level" );

        for( var level = 0; level < measured.levels.Length; ++level ) {
            var line   = measured.levels[level];
            var energy = measured.energyOf( level );

            Assert.AreEqual( level,                                    line.level,                   $"level {level}" );
            Assert.AreEqual( measured.verticesOf( level ).Count( ),    line.vertices,                $"level {level}: vertices" );
            Assert.AreEqual( energy.links,                             line.afterPositions.links,    $"level {level}: links" );
            Assert.AreEqual( energy.position,                          line.afterPositions.position, $"level {level}: what its lattice is like when it is handed down" );
            Assert.AreEqual( energy.orientation,                       line.afterPositions.orientation, $"level {level}: what its directions are like, made again from below" );
        }

        // the fields start at random at the top level, which is a vertex for each box and is not smoothed: its
        // lattice point is within a length of it each way, in its tangent plane
        foreach( var vertex in first.verticesOf( first.hierarchy.levelCount - 1 ) ) {
            var offset = d( first.hierarchy.lattice[vertex] ) - d( first.hierarchy.positions[vertex] );

            Assert.Greater( math.length( offset ), 1e-3, $"vertex {vertex} of the top level: a lattice point of its own" );
            Assert.LessOrEqual( math.length( offset ), 0.5 * math.SQRT2 + 1e-4, $"vertex {vertex} of the top level: within a length each way" );
            Assert.AreEqual( 0.0, math.dot( d( first.hierarchy.normals[vertex] ), offset ), 1e-4, $"vertex {vertex} of the top level: in its tangent plane" );
        }

        // another seed, other fields
        var settings = settingsOf( symmetry, 0.5 );

        settings.seed = 12345;

        using var other = new Solved( built, settings );

        CollectionAssert.AreNotEqual( first.hierarchy.orientations.AsArray( ).ToArray( ), other.hierarchy.orientations.AsArray( ).ToArray( ), "another seed" );
    }

    // ---- one vertex smoothed, on fields set by hand ----

    // a flat fan of six triangles round the origin, its rim a length of 0.5 away, with a direction and a lattice point
    // set at each of its seven vertices: the direction at so many degrees from the x axis, and the lattice point at
    // an offset in lengths, at a length of 2. The middle is smoothed once, and what it comes to is given: its
    // direction in degrees, and its lattice point in lengths
    private static void onAFan( Symmetry symmetry, bool positions, double[] degrees, double2[] offsets, Action<double, double2> then ) {
        const float rho = 2.0f;

        var built = new HandBuiltMesh( );

        // the middle first, so that it is vertex 0, and the rim after it in its order
        for( var k = 0; k < 6; ++k ) {
            built.triangle( Vector3.zero, HandBuiltMesh.onRing( k, 6, 0.5f, 0.0f ), HandBuiltMesh.onRing( k + 1, 6, 0.5f, 0.0f ) );
        }

        using var nativePositions = built.nativePositions( );
        using var normals         = built.nativeNormals( );
        using var triangles       = built.nativeTriangles( );
        using var lineCount       = new NativeArray<int>( 1, Allocator.TempJob );
        using var mesh            = new BurstIsotropic.FeatureMesh( nativePositions.Length, triangles.Length, Allocator.TempJob );
        using var hierarchy       = new BurstFields.Hierarchy( Allocator.TempJob );

        new BurstMeshSimplifier.BuildJob { positions = nativePositions, normals = normals, triangles = triangles, mesh = mesh.mesh }.Run( );

        new BurstFeatures.SkeletonJob { cosine = math.cos( math.radians( featureAngle ) ), mesh = mesh.mesh, edgeLines = mesh.edgeLines, corners = mesh.corners, lineCount = lineCount }.Run( );

        var report = new BurstFields.Report( );

        BurstFields.graph( mesh, hierarchy, true, ref report );

        new BurstFields.HierarchyJob { hierarchy = hierarchy }.Run( );

        Assert.AreEqual( 0, hierarchy.held[0], "the middle is not held" );
        Assert.AreEqual( 6, hierarchy.linkStarts[1] - hierarchy.linkStarts[0], "the middle's neighbours" );

        var orientations = hierarchy.orientations;
        var lattice      = hierarchy.lattice;

        for( var vertex = 0; vertex < 7; ++vertex ) {
            var angle = math.radians( degrees[vertex] );

            orientations[vertex] = new float3( ( float )math.cos( angle ), ( float )math.sin( angle ), 0.0f );
            lattice[vertex]      = new float3( ( float )offsets[vertex].x * rho, ( float )offsets[vertex].y * rho, 0.0f );
        }

        // the middle is the only vertex of its colour, since every other is its neighbour
        var colour = Enumerable.Range( hierarchy.levelColours[0], hierarchy.levelColours[1] - hierarchy.levelColours[0] ).Single( ( c ) => hierarchy.order[hierarchy.colourStarts[c]] == 0 );

        Assert.AreEqual( 1, hierarchy.colourStarts[colour + 1] - hierarchy.colourStarts[colour], "vertices of the middle's colour" );

        new BurstFields.ColourJob { view = new BurstFields.View( hierarchy ), start = hierarchy.colourStarts[colour], positions = positions, rho = rho, symmetry = symmetry }.Schedule( 1, 1 ).Complete( );

        for( var vertex = 1; vertex < 7; ++vertex ) {
            var angle = math.radians( degrees[vertex] );

            Assert.AreEqual( new float3( ( float )math.cos( angle ), ( float )math.sin( angle ), 0.0f ), orientations[vertex], $"vertex {vertex} is left as it was" );
        }

        then( math.degrees( math.atan2( ( double )orientations[0].y, orientations[0].x ) ), new double2( lattice[0].x, lattice[0].y ) / rho );
    }

    [Test]
    public void aDirectionIsSmoothedAgainstItsNeighboursOneAfterTheOther( [Values] Symmetry symmetry ) {
        // the neighbours in their order, each within a quarter of a step of the sum so far, and each given at another
        // of its turns: the sum keeps its own length from one to the next (equation 3), which is 2 cos 20 after two
        // that are 40 degrees apart, and not 2
        var step     = 360.0 / ( int )symmetry;
        var near     = new[] { 0.0, 40.0 * step / 90.0, -20.0 * step / 90.0, 10.0 * step / 90.0, -5.0 * step / 90.0, 20.0 * step / 90.0 };
        var degrees  = new[] { 77.0, near[0] + step, near[1] - step, near[2] + ( 2.0 * step ), near[3], near[4] - ( 2.0 * step ), near[5] + step };
        var sum      = double2.zero;

        foreach( var angle in near ) {
            sum += new double2( math.cos( math.radians( angle ) ), math.sin( math.radians( angle ) ) );
        }

        var expected = math.degrees( math.atan2( sum.y, sum.x ) );

        onAFan( symmetry, false, degrees, new double2[7], ( direction, point ) => {
            // at whichever turn the first neighbour was given
            var turns = ( direction - expected ) / step;

            Assert.AreEqual( 0.0, ( turns - math.round( turns ) ) * step, 2e-3, $"the direction, {direction} degrees, against {expected}" );
        } );
    }

    [Test]
    public void aLatticePointIsSmoothedToTheMeanOfItsNeighbours( [Values] Symmetry symmetry ) {
        // every direction the same, and the neighbours' lattices shifted a little from one another: the mean of the
        // six shifts, at the lattice point nearest the middle, which is at the origin
        var shifts  = new[] { new double2( 0.10, 0.02 ), new double2( 0.22, -0.04 ), new double2( -0.06, 0.12 ), new double2( 0.14, 0.2 ), new double2( 0.0, -0.1 ), new double2( 0.2, 0.16 ) };
        var first   = new double2( 1.0, 0.0 );
        var second  = symmetry == Symmetry.Quads ? new double2( 0.0, 1.0 ) : new double2( 0.5, math.sqrt( 3.0 ) / 2.0 );
        var offsets = new double2[7];
        var mean    = double2.zero;

        // the middle starts half a cell from them all, and each neighbour is given at another point of its lattice
        offsets[0] = new double2( 0.45, 0.4 );

        for( var vertex = 1; vertex < 7; ++vertex ) {
            offsets[vertex]  = shifts[vertex - 1] + ( first * ( ( vertex % 3 ) - 1 ) ) + ( second * ( ( vertex % 2 ) * 2 ) );
            mean            += shifts[vertex - 1] / 6.0;
        }

        Assert.Greater( math.distance( mean, shifts[5] ), 0.1, "the mean is not the last neighbour's" );

        onAFan( symmetry, true, new double[7], offsets, ( direction, point ) => {
            Assert.Less( math.distance( mean, point ), 1e-5, $"the lattice point {point}, against {mean}" );
        } );
    }

    // ---- what the fields are like, on fields set by hand ----

    // one flat triangle with a direction and a lattice point set at each of its vertices, at a length of 2: the
    // direction at so many degrees from the x axis, and the lattice point so many lengths along x from the origin
    private static void onATriangle( Symmetry symmetry, double3 degrees, double3 offsets, Action<BurstFields.Energy, BurstFields.Singularities> then ) {
        const float rho = 2.0f;

        var built = new HandBuiltMesh( ).triangle( Vector3.zero, new Vector3( 0.6f, 0.0f, 0.0f ), new Vector3( 0.0f, 0.6f, 0.0f ) );

        using var positions     = built.nativePositions( );
        using var normals       = built.nativeNormals( );
        using var triangles     = built.nativeTriangles( );
        using var lineCount     = new NativeArray<int>( 1, Allocator.TempJob );
        using var mesh          = new BurstIsotropic.FeatureMesh( positions.Length, triangles.Length, Allocator.TempJob );
        using var hierarchy     = new BurstFields.Hierarchy( Allocator.TempJob );
        using var energy        = new NativeArray<BurstFields.Energy>( 1, Allocator.TempJob );
        using var singularities = new NativeArray<BurstFields.Singularities>( 1, Allocator.TempJob );

        new BurstMeshSimplifier.BuildJob { positions = positions, normals = normals, triangles = triangles, mesh = mesh.mesh }.Run( );

        new BurstFeatures.SkeletonJob { cosine = math.cos( math.radians( featureAngle ) ), mesh = mesh.mesh, edgeLines = mesh.edgeLines, corners = mesh.corners, lineCount = lineCount }.Run( );

        var report = new BurstFields.Report( );

        BurstFields.graph( mesh, hierarchy, true, ref report );

        new BurstFields.HierarchyJob { hierarchy = hierarchy }.Run( );

        var orientations = hierarchy.orientations;
        var lattice      = hierarchy.lattice;

        for( var vertex = 0; vertex < 3; ++vertex ) {
            var angle = math.radians( degrees[vertex] );

            orientations[vertex] = new float3( ( float )math.cos( angle ), ( float )math.sin( angle ), 0.0f );
            lattice[vertex]      = new float3( ( float )offsets[vertex] * rho, 0.0f, 0.0f );
        }

        var view = new BurstFields.View( hierarchy );

        new BurstFields.EnergyJob { view = view, level = 0, positions = true, rho = rho, symmetry = symmetry, energy = energy }.Run( );

        new BurstFields.SingularitiesJob {
            view = view, rho = rho, symmetry = symmetry, triangles = mesh.mesh.triangles.AsArray( ), subMeshes = mesh.mesh.subMeshes.AsArray( ), singularities = singularities
        }.Run( );

        then( energy[0], singularities[0] );
    }

    [Test]
    public void directionsThatTurnRoundATriangleAreAnIrregularPoint( [Values] Symmetry symmetry ) {
        // ten degrees apart: the same turn of each is nearest all round
        onATriangle( symmetry, new double3( 0.0, 10.0, 20.0 ), double3.zero, ( energy, singularities ) => {
            Assert.AreEqual( 0, singularities.orientation, "irregular points of the directions" );
            Assert.AreEqual( 6, energy.links, "links" );

            // angles of 10, 10 and 20 degrees, squared
            Assert.AreEqual( 200.0, energy.orientation, 1e-2, "the energy of the directions" );
        } );

        // a third of a step apart: from the last to the first the nearest is a step on, so once round is a step
        var step = 360.0 / ( int )symmetry;

        onATriangle( symmetry, new double3( 0.0, step / 3.0, 2.0 * step / 3.0 ), double3.zero, ( energy, singularities ) => {
            Assert.AreEqual( 1, singularities.orientation, "irregular points of the directions" );
            Assert.AreEqual( 0, singularities.position,    "irregular points of the lattice are not looked for there" );

            Assert.AreEqual( step * step / 9.0, energy.orientation, 1e-2, "the energy of the directions" );
        } );
    }

    [Test]
    public void latticesThatShiftRoundATriangleAreAnIrregularPoint( [Values] Symmetry symmetry ) {
        // the lattice of each vertex shifted a little from the one before: all three are the same steps apart
        onATriangle( symmetry, double3.zero, new double3( 0.0, 0.1, 0.2 ), ( energy, singularities ) => {
            Assert.AreEqual( 0, singularities.orientation, "irregular points of the directions" );
            Assert.AreEqual( 0, singularities.position,    "irregular points of the lattice" );

            // 0.1, 0.1 and 0.2 lengths apart, squared
            Assert.AreEqual( 0.02, energy.position, 1e-5, "the energy of the lattice" );
        } );

        // shifted by two fifths each: from the last to the first the nearest is a step on, so once round is a step
        onATriangle( symmetry, double3.zero, new double3( 0.0, 0.4, 0.8 ), ( energy, singularities ) => {
            Assert.AreEqual( 0, singularities.orientation, "irregular points of the directions" );
            Assert.AreEqual( 1, singularities.position,    "irregular points of the lattice" );

            // 0.4, 0.4 and 0.2 lengths apart, squared
            Assert.AreEqual( 0.12, energy.position, 1e-5, "the energy of the lattice" );
        } );
    }

    [Test]
    public void aMeshWithoutTrianglesHasNoFields( [Values] Symmetry symmetry ) {
        using var solved = new Solved( new HandBuiltMesh( ), settingsOf( symmetry, 1.0, measure: true ) );

        Assert.AreEqual( 0, solved.report.vertices,       "vertices" );
        Assert.AreEqual( 0, solved.report.triangles,      "triangles" );
        Assert.AreEqual( 1, solved.hierarchy.levelCount,  "levels" );
        Assert.AreEqual( 0, solved.hierarchy.vertexCount, "vertices of the hierarchy" );
    }

    // ---- on contours ----

    public enum Contour {
        Uniform,
        Occupancy
    }

    [Test]
    public void aContourHasFieldsTheSameEveryTime( [Values] Contour contour, [Values] bool simplified, [Values] Symmetry symmetry ) {
        Voxelizer voxelizer = contour == Contour.Uniform
            ? this.createUniformVoxelizer( 4, ImplementationType.Burst, QEFSolverType.SVD, turnedShapes )
            : this.createVoxelizer<OccupancyDualContouring>( 4, QEFSolverType.SVD, turnedShapes );

        voxelizer.implementationType   = ImplementationType.Burst;
        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = false;

        var contoured = voxelizer.voxelize( voxelizer.GetComponentsInChildren<Volume>( ) ).visit( ( cpu ) => cpu.mesh, ( gpu ) => voxelizer.readMesh( gpu ) );
        var mesh      = contoured;

        if( simplified ) {
            var simplification = MeshSimplification.of( contoured, voxelizer.simplificationError * voxelizer.cellSize, voxelizer.lineQuadricWeight );

            mesh = simplification.meshAt( simplification.levelOf( 0.0f ) );
        }

        BurstMeshSimplifier.read( mesh, Allocator.TempJob, out var positions, out var normals, out var triangles );

        try {
            var kept     = Remeshing.of( mesh, voxelizer.cellSize );
            var length   = kept.lengthOf( Remeshing.Technique.FieldAligned, 1.0f, symmetry == Symmetry.Quads ? BurstFieldAligned.Faces.QuadDominant : BurstFieldAligned.Faces.Triangles );
            var settings = settingsOf( symmetry, length, measure: true );

            using var first  = new Solved( positions, normals, triangles, settings );
            using var second = new Solved( positions, normals, triangles, settings );

            assertFieldsAreWellFormed( first );
            assertTheSameFields( first, second, "twice" );

            Assert.AreEqual( positions.Length, first.report.inputVertices, "the vertices given" );
            Assert.AreEqual( 0, first.report.sheets.nonManifoldEdges, "edges of more than two triangles left by the sheets" );
            Assert.AreEqual( 0, first.report.leftOut, "vertices left out" );
            Assert.Greater( first.report.held, 0, "held: the line between the two materials" );
            Assert.Less( first.hierarchy.levelCount, BurstFields.mostLevels, "levels" );

            // the mesh the fields are on is the contour's surface: its area
            var area = 0.0;

            for( var triangle = 0; triangle < triangles.Length; ++triangle ) {
                area += 0.5 * math.length( math.cross( d( positions[triangles[triangle].b] ) - d( positions[triangles[triangle].a] ), d( positions[triangles[triangle].c] ) - d( positions[triangles[triangle].a] ) ) );
            }

            var fieldMesh = first.fields.mesh.mesh;
            var fieldArea = 0.0;

            for( var triangle = 0; triangle < fieldMesh.triangleCount; ++triangle ) {
                var corners = fieldMesh.triangles[triangle];

                fieldArea += 0.5 * math.length( math.cross( fieldMesh.positions[corners.y] - fieldMesh.positions[corners.x], fieldMesh.positions[corners.z] - fieldMesh.positions[corners.x] ) );
            }

            Assert.AreEqual( area, fieldArea, 1e-6 * area, "area" );
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );

            if( mesh != contoured ) {
                Object.DestroyImmediate( mesh );
            }

            Object.DestroyImmediate( contoured );
        }
    }

}
