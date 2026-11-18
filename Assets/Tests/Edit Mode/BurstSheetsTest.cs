using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

using Object             = UnityEngine.Object;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;

// a mesh split into sheets for the two field techniques (BurstSheets; Documents/Plans/Remeshing.md, decision 4 and
// finding 18), on meshes built by hand whose sheets are known, and on the contours that have edges of four triangles
public class BurstSheetsTest : TopologyTest {

    private sealed class Sheets : IDisposable {

        public readonly HandBuiltMesh                  built;
        public readonly NativeList<float3>             positions = new( Allocator.TempJob );
        public readonly NativeList<float3>             normals   = new( Allocator.TempJob );
        public readonly NativeList<BurstMesh.Triangle> triangles = new( Allocator.TempJob );
        public readonly NativeList<int>                sources   = new( Allocator.TempJob );
        public readonly BurstSheets.Report             report;
        public readonly MeshTopology                   topology;

        // for every triangle of the sheets, the piece it is in, numbered from 0
        public readonly int[] pieces;
        public readonly int   pieceCount;

        public Sheets( HandBuiltMesh built ) {
            this.built = built;

            using var positions = built.nativePositions( );
            using var normals   = built.nativeNormals( );
            using var triangles = built.nativeTriangles( );

            this.report = BurstSheets.split( positions, normals, triangles, this.positions, this.normals, this.triangles, this.sources );

            var mesh = BurstMesh.createMesh( this.positions.AsArray( ), this.normals.AsArray( ), this.triangles.AsArray( ) );

            this.topology = MeshTopology.of( mesh );

            Object.DestroyImmediate( mesh );

            // triangles that share a vertex are of one piece
            var parents = Enumerable.Range( 0, this.positions.Length ).ToArray( );

            int find( int vertex ) {
                while( parents[vertex] != vertex ) {
                    vertex = parents[vertex] = parents[parents[vertex]];
                }
                return vertex;
            }

            foreach( var triangle in this.triangles ) {
                parents[find( triangle.a )] = find( triangle.b );
                parents[find( triangle.b )] = find( triangle.c );
            }

            var numbers = new Dictionary<int, int>( );

            this.pieces = new int[this.triangles.Length];

            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                var root = find( this.triangles[triangle].a );

                if( !numbers.TryGetValue( root, out var number ) ) {
                    numbers.Add( root, number = numbers.Count );
                }
                this.pieces[triangle] = number;
            }

            this.pieceCount = numbers.Count;
        }

        public void Dispose( ) {
            this.positions.Dispose( );
            this.normals.Dispose( );
            this.triangles.Dispose( );
            this.sources.Dispose( );
        }

        // the positions of the vertices of a piece
        public IEnumerable<float3> positionsOf( int piece ) {
            for( var triangle = 0; triangle < this.triangles.Length; ++triangle ) {
                if( this.pieces[triangle] == piece ) {
                    yield return this.positions[this.triangles[triangle].a];
                    yield return this.positions[this.triangles[triangle].b];
                    yield return this.positions[this.triangles[triangle].c];
                }
            }
        }

        // every vertex of the sheets is where the vertex of the mesh it came from is, and every triangle is the
        // triangle of the mesh in its place, corner for corner
        public void assertTheSameSurface( ) {
            Assert.AreEqual( this.positions.Length, this.sources.Length, "a source for every vertex" );

            for( var vertex = 0; vertex < this.positions.Length; ++vertex ) {
                Assert.AreEqual( ( float3 )this.built.positions[this.sources[vertex]], this.positions[vertex], $"vertex {vertex} is where its source is" );
            }

            var kept = this.built.triangles.Where( ( triangle ) => triangle.a != triangle.b && triangle.b != triangle.c && triangle.a != triangle.c ).ToArray( );

            Assert.AreEqual( kept.Length, this.triangles.Length, "triangles" );

            for( var triangle = 0; triangle < kept.Length; ++triangle ) {
                var sheet = this.triangles[triangle];

                Assert.AreEqual( kept[triangle].a,            this.sources[sheet.a],  $"triangle {triangle}, first corner" );
                Assert.AreEqual( kept[triangle].b,            this.sources[sheet.b],  $"triangle {triangle}, second corner" );
                Assert.AreEqual( kept[triangle].c,            this.sources[sheet.c],  $"triangle {triangle}, third corner" );
                Assert.AreEqual( kept[triangle].subMeshIndex, sheet.subMeshIndex,     $"triangle {triangle}, sub-mesh" );
            }
        }

    }

    [Test]
    public void aManifoldComesBackAsItWas( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, new Vector3Int( 2, 1, 3 ), ( centre, axis, side ) => axis );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 0, sheets.report.edges,         "edges of more than two triangles" );
        Assert.AreEqual( 0, sheets.report.addedVertices, "vertices added" );

        CollectionAssert.AreEqual( built.positions.Select( ( position ) => ( float3 )position ).ToArray( ), sheets.positions.AsArray( ).ToArray( ), "positions" );
        CollectionAssert.AreEqual( built.triangles.ToArray( ), sheets.triangles.AsArray( ).ToArray( ), "triangles" );
        CollectionAssert.AreEqual( Enumerable.Range( 0, built.positions.Count ).ToArray( ), sheets.sources.AsArray( ).ToArray( ), "sources" );
    }

    [Test]
    public void twoBoxesThatShareAnEdgeComeApartIntoTwo( ) {
        // the edge from (1, 1, 0) to (1, 1, 1) has two triangles of each box, and solid on two sides of it
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one ).box( new Vector3Int( 1, 1, 0 ), Vector3Int.one );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 1, sheets.report.edges,            "edges of more than two triangles" );
        Assert.AreEqual( 1, sheets.report.acrossSolid,      "paired across the solid" );
        Assert.AreEqual( 0, sheets.report.oppositeWinding,  "paired by winding" );
        Assert.AreEqual( 0, sheets.report.repaired,         "repaired" );
        Assert.AreEqual( 0, sheets.report.unpaired,         "unpaired" );
        Assert.AreEqual( 2, sheets.report.addedVertices,    "vertices added: the two ends of the edge" );
        Assert.AreEqual( 0, sheets.report.nonManifoldEdges, "edges of more than two triangles left" );
        Assert.AreEqual( 0, sheets.report.newOpenEdges,     "open edges made" );

        sheets.assertTheSameSurface( );

        // two closed boxes: across the air between them it would be one surface
        Assert.AreEqual( 2,  sheets.pieceCount,                   "pieces" );
        Assert.AreEqual( 0,  sheets.topology.openEdges,           "open edges" );
        Assert.AreEqual( 0,  sheets.topology.nonManifoldEdges,    "edges of more than two triangles" );
        Assert.AreEqual( 0,  sheets.topology.pinchedVertices,     "pinched vertices" );
        Assert.AreEqual( 16, sheets.topology.vertices,            "vertices" );
        Assert.AreEqual( 4,  sheets.topology.eulerCharacteristic, "two spheres" );

        for( var piece = 0; piece < 2; ++piece ) {
            var positions = sheets.positionsOf( piece ).ToArray( );
            var first     = positions.All( ( position ) => position.x <= 1.0f && position.y <= 1.0f );
            var second    = positions.All( ( position ) => position.x >= 1.0f && position.y >= 1.0f );

            Assert.IsTrue( first || second, $"piece {piece} is one of the boxes" );
        }
    }

    [Test]
    public void twoBoxesThatTouchInAVertexComeApartIntoTwo( ) {
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one ).box( Vector3Int.one, Vector3Int.one );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 0, sheets.report.edges,         "edges of more than two triangles" );
        Assert.AreEqual( 1, sheets.report.addedVertices, "vertices added: the one they touch in" );

        sheets.assertTheSameSurface( );

        Assert.AreEqual( 2,  sheets.pieceCount,                   "pieces" );
        Assert.AreEqual( 0,  sheets.topology.pinchedVertices,     "pinched vertices" );
        Assert.AreEqual( 16, sheets.topology.vertices,            "vertices" );
        Assert.AreEqual( 4,  sheets.topology.eulerCharacteristic, "two spheres" );
    }

    // two flat sheets of unit squares that pass through each other along the y axis, both facing the same way round
    // it: one in the plane z = 0 and one in the plane x = 0, from -1 to 1 across and over the rows given along
    private static HandBuiltMesh crossing( float[] rows, float[] otherRows ) {
        var built = new HandBuiltMesh( );

        for( var row = 0; row + 1 < rows.Length; ++row )
        for( var across = -1; across < 1; ++across ) {
            built.quad( new Vector3( across, rows[row], 0 ), new Vector3( across + 1, rows[row], 0 ), new Vector3( across + 1, rows[row + 1], 0 ), new Vector3( across, rows[row + 1], 0 ) );
        }

        for( var row = 0; row + 1 < otherRows.Length; ++row )
        for( var across = -1; across < 1; ++across ) {
            built.quad( new Vector3( 0, otherRows[row], across ), new Vector3( 0, otherRows[row + 1], across ), new Vector3( 0, otherRows[row + 1], across + 1 ), new Vector3( 0, otherRows[row], across + 1 ) );
        }

        return built;
    }

    [Test]
    public void twoSheetsThatCrossAlongALineComeApartIntoTwo( ) {
        var rows  = new[] { 0.0f, 1.0f, 2.0f };
        var built = crossing( rows, rows );

        using var sheets = new Sheets( built );

        // the windings do not alternate round an edge that two sheets cross at
        Assert.AreEqual( 2, sheets.report.edges,            "edges of more than two triangles" );
        Assert.AreEqual( 0, sheets.report.acrossSolid,      "paired across the solid" );
        Assert.AreEqual( 2, sheets.report.oppositeWinding,  "paired by winding" );
        Assert.AreEqual( 0, sheets.report.repaired,         "repaired: no edge keeps its four triangles" );
        Assert.AreEqual( 0, sheets.report.unpaired,         "unpaired" );
        Assert.AreEqual( 3, sheets.report.addedVertices,    "vertices added: those of the line" );
        Assert.AreEqual( 0, sheets.report.nonManifoldEdges, "edges of more than two triangles left" );
        Assert.AreEqual( 0, sheets.report.newOpenEdges,     "open edges made" );

        sheets.assertTheSameSurface( );

        Assert.AreEqual( 2,  sheets.pieceCount,                "pieces" );
        Assert.AreEqual( 0,  sheets.topology.pinchedVertices,  "pinched vertices" );
        Assert.AreEqual( 0,  sheets.topology.windingConflicts, "edges whose two triangles run the same way" );
        Assert.AreEqual( 16, sheets.topology.openEdges,        "open edges: the rims" );
    }

    [Test]
    public void twoSheetsThatCrossAtOneEdgeAreTakenStraightThroughIt( ) {
        // the sheets share the edge from (0, 1, 0) to (0, 2, 0) and no other, and both go on past its ends. Paired by
        // winding, each half of one sheet goes with a half of the other, and round each end of the edge the four are
        // one ring that passes the edge twice: the edge would keep its four triangles. Its other pairing takes each
        // sheet straight through
        var built = crossing( new[] { 0.0f, 1.0f, 2.0f, 3.0f }, new[] { -0.5f, 1.0f, 2.0f, 3.5f } );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 1, sheets.report.edges,            "edges of more than two triangles" );
        Assert.AreEqual( 1, sheets.report.oppositeWinding,  "paired by winding" );
        Assert.AreEqual( 1, sheets.report.repaired,         "repaired" );
        Assert.AreEqual( 2, sheets.report.addedVertices,    "vertices added: the two ends of the edge" );
        Assert.AreEqual( 0, sheets.report.nonManifoldEdges, "edges of more than two triangles left" );
        Assert.AreEqual( 0, sheets.report.newOpenEdges,     "open edges made" );

        sheets.assertTheSameSurface( );

        Assert.AreEqual( 2, sheets.pieceCount,                "pieces" );
        Assert.AreEqual( 0, sheets.topology.pinchedVertices,  "pinched vertices" );
        Assert.AreEqual( 0, sheets.topology.nonManifoldEdges, "edges of more than two triangles" );

        for( var piece = 0; piece < 2; ++piece ) {
            var positions = sheets.positionsOf( piece ).ToArray( );

            Assert.IsTrue( positions.All( ( position ) => position.z == 0.0f ) || positions.All( ( position ) => position.x == 0.0f ), $"piece {piece} is one of the two flat sheets" );
        }
    }

    [Test]
    public void twoSheetsThatCrossAtOneEdgeOnTheRimOfOneAreTakenStraightThroughIt( [Values] bool theFirst ) {
        // as above, but one of the sheets is one row of squares, so that both ends of the edge are on its rim. Round
        // each end its two triangles are then joined only through the other sheet: going round from one of them
        // comes to the rim, and the other pair is found from its partner. Whichever sheet it is
        var row   = new[] { 1.0f, 2.0f };
        var rows  = new[] { -0.5f, 1.0f, 2.0f, 3.5f };
        var built = theFirst ? crossing( row, rows ) : crossing( rows, row );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 1, sheets.report.edges,            "edges of more than two triangles" );
        Assert.AreEqual( 1, sheets.report.oppositeWinding,  "paired by winding" );
        Assert.AreEqual( 1, sheets.report.repaired,         "repaired" );
        Assert.AreEqual( 2, sheets.report.addedVertices,    "vertices added: the two ends of the edge" );
        Assert.AreEqual( 0, sheets.report.nonManifoldEdges, "edges of more than two triangles left" );
        Assert.AreEqual( 0, sheets.report.newOpenEdges,     "open edges made" );

        sheets.assertTheSameSurface( );

        Assert.AreEqual( 2, sheets.pieceCount,                "pieces" );
        Assert.AreEqual( 0, sheets.topology.nonManifoldEdges, "edges of more than two triangles" );

        for( var piece = 0; piece < 2; ++piece ) {
            var positions = sheets.positionsOf( piece ).ToArray( );

            Assert.IsTrue( positions.All( ( position ) => position.z == 0.0f ) || positions.All( ( position ) => position.x == 0.0f ), $"piece {piece} is one of the two flat sheets" );
        }
    }

    [Test]
    public void twoSheetsThatMeetRoundOneEndOfAnEdgeOnlyAreLeftAsTheyWerePaired( ) {
        // the sheets share the edge from (0, 1, 0) to (0, 2, 0), and both end at its far end. Round its near end the
        // four triangles are one ring that passes the edge twice, and round its far end they are two rows: the edge
        // has two triangles in each of two edges of the sheets, and the pairing is left as it is. No contour measured
        // has such an edge (step 4)
        var built = crossing( new[] { 0.0f, 1.0f, 2.0f }, new[] { -0.5f, 1.0f, 2.0f } );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 1, sheets.report.edges,            "edges of more than two triangles" );
        Assert.AreEqual( 1, sheets.report.oppositeWinding,  "paired by winding" );
        Assert.AreEqual( 0, sheets.report.repaired,         "repaired" );
        Assert.AreEqual( 1, sheets.report.addedVertices,    "vertices added: the far end of the edge" );
        Assert.AreEqual( 0, sheets.report.nonManifoldEdges, "edges of more than two triangles left" );
        Assert.AreEqual( 0, sheets.report.newOpenEdges,     "open edges made" );

        sheets.assertTheSameSurface( );

        Assert.AreEqual( 1, sheets.pieceCount,                "pieces: joined at the near end" );
        Assert.AreEqual( 0, sheets.topology.nonManifoldEdges, "edges of more than two triangles" );
        Assert.AreEqual( 0, sheets.topology.pinchedVertices,  "pinched vertices" );
    }

    [Test]
    public void anEdgeOfThreeTrianglesIsCountedAndItsOddTriangleLeftOpen( ) {
        // a box with a fin on one of its edges
        var built = new HandBuiltMesh( ).box( Vector3Int.zero, Vector3Int.one ).triangle( new Vector3( 1, 1, 0 ), new Vector3( 2, 2, 0.5f ), new Vector3( 1, 1, 1 ) );

        using var sheets = new Sheets( built );

        Assert.AreEqual( 1, sheets.report.edges,            "edges of more than two triangles" );
        Assert.AreEqual( 1, sheets.report.unpaired,         "unpaired" );
        Assert.AreEqual( 2, sheets.report.addedVertices,    "vertices added: the fin's on the edge" );
        Assert.AreEqual( 0, sheets.report.nonManifoldEdges, "edges of more than two triangles left" );
        Assert.AreEqual( 1, sheets.report.newOpenEdges,     "open edges made: the fin's on the edge" );

        sheets.assertTheSameSurface( );

        // the box stays closed: its two triangles at the edge are joined round each end of it
        Assert.AreEqual( 2, sheets.pieceCount,         "pieces" );
        Assert.AreEqual( 3, sheets.topology.openEdges, "open edges: the fin's three" );
    }

    [Test]
    public void aTriangleThatUsesAVertexTwiceIsLeftOut( ) {
        var built = new HandBuiltMesh( ).patch( 2, 2 ).triangle( Vector3.zero, Vector3.zero, new Vector3( 1, 0, 0 ) );

        using var sheets = new Sheets( built );

        Assert.AreEqual( built.triangles.Count - 1, sheets.triangles.Length, "triangles" );
        Assert.AreEqual( 0,                         sheets.report.edges,     "edges of more than two triangles" );

        sheets.assertTheSameSurface( );
    }

    // the contours with edges of four triangles (finding 18), contoured and simplified: Uniform's of the turned and
    // the new shapes, whose sheets cross, at 32^3 and at 64^3, where an edge needs its other pairing
    public enum Crossing {
        TurnedShapes,
        NewShapes
    }

    [Test]
    public void theContoursWithEdgesOfFourTrianglesComeApartIntoManifolds(
        [Values] Crossing crossing, [Values( 5, 6 )] int cellsExponent, [Values] bool simplified
    ) {
        var voxelizer = this.createUniformVoxelizer( cellsExponent, ImplementationType.Burst, QEFSolverType.SVD, crossing == Crossing.TurnedShapes ? turnedShapes : newShapes );

        voxelizer.keepDebugInformation = false;
        voxelizer.meshSimplification   = false;

        var contour = voxelizer.voxelize( voxelizer.GetComponentsInChildren<Volume>( ) ).visit( ( cpu ) => cpu.mesh, ( gpu ) => voxelizer.readMesh( gpu ) );
        var mesh    = contour;

        if( simplified ) {
            var simplification = MeshSimplification.of( contour, voxelizer.simplificationError * voxelizer.cellSize, voxelizer.lineQuadricWeight );

            mesh = simplification.meshAt( simplification.levelOf( 0.0f ) );
        }

        BurstMeshSimplifier.read( mesh, Allocator.TempJob, out var positions, out var normals, out var triangles );

        using var sheetPositions = new NativeList<float3>( Allocator.TempJob );
        using var sheetNormals   = new NativeList<float3>( Allocator.TempJob );
        using var sheetTriangles = new NativeList<BurstMesh.Triangle>( Allocator.TempJob );
        using var sources        = new NativeList<int>( Allocator.TempJob );

        Mesh sheetMesh = null;

        try {
            var before = MeshTopology.of( mesh );
            var report = BurstSheets.split( positions, normals, triangles, sheetPositions, sheetNormals, sheetTriangles, sources );

            sheetMesh = BurstMesh.createMesh( sheetPositions.AsArray( ), sheetNormals.AsArray( ), sheetTriangles.AsArray( ) );

            var after = MeshTopology.of( sheetMesh );

            Assert.Greater( before.nonManifoldEdges, 0, "the contour has edges of more than two triangles" );

            Assert.AreEqual( before.nonManifoldEdges, report.edges,                                      "every such edge is seen" );
            Assert.AreEqual( report.edges,            report.acrossSolid + report.oppositeWinding,       "and paired" );
            Assert.AreEqual( 0,                       report.unpaired,                                   "unpaired" );
            Assert.AreEqual( 0,                       report.nonManifoldEdges,                           "edges of more than two triangles left" );
            Assert.AreEqual( 0,                       report.newOpenEdges,                               "open edges made" );
            Assert.AreEqual( 0,                       after.nonManifoldEdges,                            "edges of more than two triangles" );
            Assert.AreEqual( 0,                       after.pinchedVertices,                             "pinched vertices" );
            Assert.AreEqual( before.openEdges,        after.openEdges,                                   "open edges" );
            Assert.AreEqual( 0,                       after.windingConflicts,                            "edges whose two triangles run the same way" );
            Assert.AreEqual( before.triangles,        after.triangles,                                   "triangles" );
            Assert.AreEqual( before.vertices + report.addedVertices, after.vertices,                     "vertices" );

            if( cellsExponent == 6 ) {
                Assert.Greater( report.repaired, 0, "at 64^3 an edge needs its other pairing" );
            }

            for( var vertex = 0; vertex < sheetPositions.Length; ++vertex ) {
                Assert.AreEqual( positions[sources[vertex]], sheetPositions[vertex], $"vertex {vertex} is where its source is" );
            }
        }
        finally {
            positions.Dispose( );
            normals.Dispose( );
            triangles.Dispose( );

            Object.DestroyImmediate( sheetMesh );

            if( mesh != contour ) {
                Object.DestroyImmediate( mesh );
            }

            Object.DestroyImmediate( contour );
        }
    }

}
