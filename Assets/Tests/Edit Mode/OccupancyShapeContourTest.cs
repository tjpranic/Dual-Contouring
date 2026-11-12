using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;
using Turn               = ShapeContourTest.Turn;

// where Occupancy-Based Dual Contouring puts its vertices on shapes whose surface is known: on a plane, on a cube and
// on each kind of volume, upright and turned, for each implementation in the fixtures at the end. Distances are in
// cells of the grid of 16^3.
//
// The tolerances are what the algorithm of the paper achieves, measured, and they say where it is exact and where it is
// not:
// - Flat faces, and edges that every face of the grid sees as a single bend, are exact: the upright cuboid and the
//   upright hexagonal prism.
// - Curved surfaces are approximated by planes through points on them, far closer than by tangent planes.
// - Where a face of the grid sees three faces of a shape, the line across it bends twice, which lemma 5.1 does not cover.
//   One of the planes there is wrong, and a corner where three or more faces meet is missed by up to half a cell.
// - A vertex the QEF puts outside its cell is moved back in, off the planes it was placed by. On turned shapes with
//   edges that is one vertex in six to ten, by up to 0.4 of a cell
public abstract class OccupancyShapeContourTest : TopologyTest {

    private const int   resolution = 4;
    private const float cell       = 1.0f / 16;

    public static readonly DensityFunction.Type[] cornered = { DensityFunction.Type.Cuboid, DensityFunction.Type.Cone, DensityFunction.Type.Pyramid, DensityFunction.Type.Octahedron, DensityFunction.Type.HexagonalPrism };
    public static readonly DensityFunction.Type[] edged    = { DensityFunction.Type.Cuboid, DensityFunction.Type.Cylinder, DensityFunction.Type.Cone, DensityFunction.Type.Pyramid, DensityFunction.Type.Octahedron, DensityFunction.Type.HexagonalPrism };

    protected static Vector3 extentsOf( DensityFunction.Type type ) {
        return type == DensityFunction.Type.Ellipsoid ? new Vector3( 0.3f, 0.25f, 0.2f ) : ShapeContourTest.extentsOf( type );
    }

    protected OccupancyDualContouring create( DensityFunction.Type type, Turn turn ) {
        var shape = new Shape( type, ShapeContourTest.center, extentsOf( type ), Combination.Union, MaterialIndex.Material0 ).rotated( ShapeContourTest.rotationOf( turn ) );

        return this.createVoxelizer<OccupancyDualContouring>( resolution, QEFSolverType.SVD, new[] { shape } );
    }

    // section 5.4 keeps every vertex inside its cell, where the reference implementation has it a thousandth of a cell
    // from the faces, and a vertex of a cell with several is kept on its own side of the planes between them. A vertex
    // the QEF put outside was moved back, and is on one of those inner faces or planes
    private static HashSet<SurfaceExtractor.Voxel> findMovedIntoTheirCells( SurfaceExtractor.Voxel[] voxels ) {
        const float reach = 1.5f * BurstOccupancy.margin;

        var moved = new HashSet<SurfaceExtractor.Voxel>( );

        foreach( var cell in voxels.GroupBy( ( voxel ) => voxel.center ) ) {
            // the debug view lists the vertices of a cell in the order the table numbers them. A cell that took its
            // vertices from its complement has as many as that has
            var vertices     = cell.ToArray( );
            var solidCorners = DualMarchingCubes.solidCornersOf( vertices[0].corners.Select( ( corner ) => corner.materialIndex ).ToArray( ) );
            var lookup       = DualMarchingCubes.lookupVertexEdges( solidCorners ).Length == vertices.Length ? solidCorners : DualMarchingCubes.complement( solidCorners );

            for( var vertex = 0; vertex < vertices.Length; ++vertex ) {
                var inCell = ( vertices[vertex].vertex - vertices[vertex].minimum ).divide( vertices[vertex].size );

                var atFace = Enumerable.Range( 0, 3 ).Any( ( axis ) => inCell[axis] < reach || inCell[axis] > 1.0f - reach );

                var atPlane = Enumerable.Range( 0, vertices.Length ).Any(
                    ( other ) => DualMarchingCubes.lookupSeparation( lookup, vertex, other, out var normal, out var twiceOffset ) &&
                                 Vector3.Dot( new Vector3( normal.x, normal.y, normal.z ), inCell ) > ( twiceOffset * 0.5f ) - reach
                );

                if( atFace || atPlane ) {
                    moved.Add( vertices[vertex] );
                }
            }
        }

        return moved;
    }

    // the corners of a cuboid in its own frame, which ShapeContourTest leaves to RotatedCubeTest
    private static Vector3[] cornersOf( DensityFunction.Type type, Vector3 extents ) {
        if( type != DensityFunction.Type.Cuboid ) {
            return ShapeContourTest.cornersOf( type, extents );
        }

        return Enumerable.Range( 0, 8 ).Select(
            ( corner ) => new Vector3( ( corner & 1 ) == 0 ? -extents.x : extents.x, ( corner & 2 ) == 0 ? -extents.y : extents.y, ( corner & 4 ) == 0 ? -extents.z : extents.z )
        ).ToArray( );
    }

    private static Vector3[] edgePointsOf( DensityFunction.Type type, Vector3 extents ) {
        if( type != DensityFunction.Type.Cuboid ) {
            return ShapeContourTest.edgePointsOf( type, extents );
        }

        var corners = cornersOf( type, extents );

        // corners whose numbers differ in one bit share an edge
        return Enumerable.Range( 0, 8 ).SelectMany(
            ( corner ) => new[] { 1, 2, 4 }.Where( ( bit ) => ( corner & bit ) == 0 ).SelectMany(
                ( bit ) => Enumerable.Range( 0, 33 ).Select( ( step ) => Vector3.Lerp( corners[corner], corners[corner | bit], step / 32.0f ) )
            )
        ).ToArray( );
    }

    // a flat wall through the grid at an angle: every 1D point is on it, every crossing of a face is straight, and every
    // plane is the wall
    [Test]
    public void everyVertexOfAPlaneLiesOnIt( [Values( 0, 1, 2 )] int turn ) {
        var rotation = turn switch {
            0 => Quaternion.Euler( 30.0f, -60.0f, 75.0f ),
            1 => Quaternion.AngleAxis( 45.0f, Vector3.up ),
            _ => Quaternion.AngleAxis( 20.0f, new Vector3( 1.0f, 2.0f, 3.0f ) )
        };

        // a cube of two across, far larger than the grid, whose top is just above the middle of the grid
        var normal = rotation * Vector3.up;
        var point  = normal * 0.013f;

        var shapes = new[] { cube( point - normal, 1.0f ).rotated( rotation ) };

        var vertices = this.voxelize( this.createVoxelizer<OccupancyDualContouring>( resolution, QEFSolverType.SVD, shapes ) ).mesh.vertices;

        Assert.Greater( vertices.Length, 100, "vertices" );

        var worst = vertices.Max( ( vertex ) => Mathf.Abs( Vector3.Dot( vertex - point, normal ) ) ) / cell;

        Assert.LessOrEqual( worst, planeTolerance, "how far the farthest vertex is from the plane, in cells" );
    }

    // a vertex closer to a face of its cell than the margin is moved in by up to that much, a thousandth of a cell
    // along each axis. The farthest was 0.00057 off
    private const float planeTolerance = 0.002f;

    // the shapes whose faces and edges every face of this grid sees straight or with a single bend when they are upright
    private static bool isExact( DensityFunction.Type type, Turn turn ) {
        return turn == Turn.Upright && ( type == DensityFunction.Type.Cuboid || type == DensityFunction.Type.HexagonalPrism );
    }

    private static bool hasFlatFacesOnly( DensityFunction.Type type ) {
        return type == DensityFunction.Type.Cuboid || type == DensityFunction.Type.Pyramid || type == DensityFunction.Type.Octahedron || type == DensityFunction.Type.HexagonalPrism;
    }

    // the vertices the QEF placed inside their cells, against the shape, and the ones it placed outside and that were
    // moved back in, which is what keeping the polygons free of intersections costs
    [Test]
    public void everyVertexLiesOnTheShape( [Values] DensityFunction.Type type, [Values] Turn turn ) {
        var voxelizer = this.create( type, turn );
        var volume    = voxelizer.GetComponentInChildren<Volume>( );
        var voxels    = this.voxelize( voxelizer ).voxels.ToArray( );

        Assert.Greater( voxels.Length, 100, "vertices" );

        // every shape is an exact signed distance, so its density is how far a point is from it
        float distanceOf( SurfaceExtractor.Voxel voxel ) {
            return Mathf.Abs( volume.sample( voxel.vertex ) ) / cell;
        }

        var movedIntoTheirCells = findMovedIntoTheirCells( voxels );

        var free  = voxels.Where( ( voxel ) => !movedIntoTheirCells.Contains( voxel ) ).ToArray( );
        var moved = voxels.Where( movedIntoTheirCells.Contains ).ToArray( );

        var worstFree  = free.Max( distanceOf );
        var worstMoved = moved.Length > 0 ? moved.Max( distanceOf ) : 0.0f;
        var share      = moved.Length / ( float )voxels.Length;

        // - not moved: upright shapes with flat faces only have every such vertex within 0.0001; the farthest of the
        //   others is 0.083, near the tip of the turned cone
        // - moved: the farthest is 0.40, on the turned cone, and the most are moved on the turned octahedron, 17.7%
        var tolerance = hasFlatFacesOnly( type ) && turn == Turn.Upright ? 0.001f : 0.1f;

        Assert.LessOrEqual( worstFree,  tolerance, $"how far the farthest of the {free.Length} vertices that were not moved is from the shape, in cells" );
        Assert.LessOrEqual( worstMoved, 0.45f,     $"how far the farthest of the {moved.Length} vertices that were moved into their cells is from the shape, in cells" );
        Assert.LessOrEqual( share,      0.2f,      $"the share of the vertices that were moved into their cells, {moved.Length} of {voxels.Length}" );
    }

    // the cell around a corner sees the faces that meet there, and its vertex goes where they meet, when every face of
    // the grid sees those faces straight or with a single bend. Otherwise a corner can be left without a vertex: the
    // nearest was up to 0.44 of a cell away, on the turned cuboid and the turned pyramid
    [Test]
    public void everyCornerHasAVertex( [ValueSource( nameof( cornered ) )] DensityFunction.Type type, [Values] Turn turn ) {
        var voxelizer = this.create( type, turn );
        var volume    = voxelizer.GetComponentInChildren<Volume>( );
        var vertices  = this.voxelize( voxelizer ).mesh.vertices;

        var nearest = cornersOf( type, extentsOf( type ) ).Select(
            ( corner ) => ( corner, distance: vertices.Min( ( vertex ) => Vector3.Distance( vertex, ShapeContourTest.toWorld( volume, corner ) ) ) / cell )
        ).OrderByDescending( ( item ) => item.distance ).First( );

        Assert.LessOrEqual( nearest.distance, isExact( type, turn ) ? 0.001f : 0.5f, $"the nearest vertex to the corner at {nearest.corner}, in the shape's frame, the farthest of its corners, in cells" );
    }

    // the mesh along the edges and rims of the shape. It runs along most of each, and leaves them near the corners that
    // are missed and at the vertices that were moved into their cells: by up to 0.56 of a cell, on the turned pyramid. The
    // worst median is 0.13, on the turned cone
    [Test]
    public void theMeshRunsAlongEveryEdge( [ValueSource( nameof( edged ) )] DensityFunction.Type type, [Values] Turn turn ) {
        var voxelizer = this.create( type, turn );
        var volume    = voxelizer.GetComponentInChildren<Volume>( );
        var mesh      = this.voxelize( voxelizer ).mesh;

        var vertices  = mesh.vertices;
        var triangles = mesh.triangles;

        var distances = edgePointsOf( type, extentsOf( type ) ).Select(
            ( local ) => distanceToMesh( ShapeContourTest.toWorld( volume, local ), vertices, triangles ) / cell
        ).OrderBy( ( distance ) => distance ).ToArray( );

        Assert.LessOrEqual( distances.Last( ),                 isExact( type, turn ) ? 0.02f : 0.6f,  "how far the mesh is from the edges at worst, in cells" );
        Assert.LessOrEqual( distances[distances.Length / 2], isExact( type, turn ) ? 0.001f : 0.15f, "how far the mesh is from the middle one of the points along the edges, in cells" );
    }

    // section 5.4: no two triangles of any shape pass through each other. Split along the same diagonal always, as
    // Manifold Dual Contouring splits them, the polygons of the turned shapes with edges do: 2 to 7 pairs of triangles
    // on each, and 16 on the upright octahedron
    [Test]
    public void noTwoTrianglesCross( [Values] DensityFunction.Type type, [Values] Turn turn ) {
        Assert.AreEqual( 0, MeshIntersections.count( this.voxelize( this.create( type, turn ) ).mesh ), "pairs of triangles that pass through each other" );
    }

    [Test]
    public void withoutTheSplitTrianglesCross( [Values( DensityFunction.Type.Cuboid, DensityFunction.Type.Octahedron, DensityFunction.Type.Cone )] DensityFunction.Type type ) {
        var voxelizer = this.create( type, Turn.Turned );

        voxelizer.splitQuads = false;

        Assert.Greater( MeshIntersections.count( this.voxelize( voxelizer ).mesh ), 0, "pairs of triangles that pass through each other without the split" );
    }

    // with vertices left where the QEF puts them, the turned shapes with edges keep those edges: no vertex is moved, the
    // farthest is a tenth as far from the shape as with the clamp, and the mesh is 0.05 of a cell from the edges at nine
    // points in ten instead of 0.2. Triangles pass through each other again, and no polygon is split into four
    [Test]
    public void withoutKeepingVerticesInTheirCellsEdgesAreSharperAndTrianglesCross( [Values( DensityFunction.Type.Cuboid, DensityFunction.Type.HexagonalPrism )] DensityFunction.Type type ) {
        var voxelizer = this.create( type, Turn.Turned );
        var volume    = voxelizer.GetComponentInChildren<Volume>( );

        ( float farthest, float alongEdges, int outside, int crossings, int onEdges ) contour( ) {
            var voxelization = this.voxelize( voxelizer );
            var mesh         = voxelization.mesh;
            var voxels       = voxelization.voxels.ToArray( );

            var vertices  = mesh.vertices;
            var triangles = mesh.triangles;

            var distances = edgePointsOf( type, extentsOf( type ) ).Select(
                ( local ) => distanceToMesh( ShapeContourTest.toWorld( volume, local ), vertices, triangles ) / cell
            ).OrderBy( ( distance ) => distance ).ToArray( );

            return (
                voxels.Max( ( voxel ) => Mathf.Abs( volume.sample( voxel.vertex ) ) ) / cell,
                distances[( int )( distances.Length * 0.9f )],
                voxels.Count( ( voxel ) => Enumerable.Range( 0, 3 ).Any( ( axis ) => voxel.vertex[axis] < voxel.minimum[axis] || voxel.vertex[axis] > voxel.maximum[axis] ) ),
                MeshIntersections.count( mesh ),
                mesh.vertexCount - voxels.Length
            );
        }

        var kept = contour( );

        voxelizer.keepVerticesInCells = false;

        var free = contour( );

        Assert.AreEqual( 0, kept.outside, "vertices outside their cells, kept in" );
        Assert.Greater( free.outside, 20, "vertices outside their cells, not kept in" );

        Assert.Greater( kept.farthest, 0.2f,     "how far the farthest vertex is from the shape, kept in, in cells" );
        Assert.LessOrEqual( free.farthest, 0.05f, "how far the farthest vertex is from the shape, not kept in, in cells" );

        Assert.Greater( kept.alongEdges, 0.1f,       "how far the mesh is from the edges at nine points in ten, kept in, in cells" );
        Assert.LessOrEqual( free.alongEdges, 0.06f, "how far the mesh is from the edges at nine points in ten, not kept in, in cells" );

        Assert.AreEqual( 0, kept.crossings, "pairs of triangles that pass through each other, kept in" );
        Assert.Greater( free.crossings, 0,  "pairs of triangles that pass through each other, not kept in" );

        Assert.AreEqual( 0, free.onEdges, "vertices of the mesh on edges of the grid, not kept in" );
    }

}

public class BurstOccupancyShapeContourTest : OccupancyShapeContourTest {

    public BurstOccupancyShapeContourTest( ) {
        this.implementation = ImplementationType.Burst;
    }

    // section 5.3: the plane at a 1D point goes through it and the two 2D points next to it. On a cube those are on the
    // face the 1D point is on, or at an edge of that face, so the plane is the face. The debug view of the Burst
    // implementation shows the normal of the plane with the edge; the GPU implementation does not keep its planes
    [Test]
    public void thePlanesOfACubeAreItsFaces( [Values] Turn turn ) {
        var voxelizer = this.create( DensityFunction.Type.Cuboid, turn );
        var volume    = voxelizer.GetComponentInChildren<Volume>( );
        var extents   = extentsOf( DensityFunction.Type.Cuboid );

        var crossing = this.voxelize( voxelizer ).edges.Where( ( edge ) => edge.intersectsContour( ) ).ToArray( );

        Assert.Greater( crossing.Length, 100, "edges that cross the contour" );

        // the angle between the normal an edge shows and that of the face of the cube its 1D point is on, in degrees
        var angles = crossing.Select( ( edge ) => {
            var local = ShapeContourTest.toLocal( volume, edge.intersection );
            var off   = new Vector3( Mathf.Abs( Mathf.Abs( local.x ) - extents.x ), Mathf.Abs( Mathf.Abs( local.y ) - extents.y ), Mathf.Abs( Mathf.Abs( local.z ) - extents.z ) );
            var axis  = off.x <= off.y && off.x <= off.z ? 0 : off.y <= off.z ? 1 : 2;

            var normal = Vector3.zero;

            normal[axis] = Mathf.Sign( local[axis] );

            return Vector3.Angle( volume.transform.rotation * normal, edge.normal );
        } ).OrderBy( ( angle ) => angle ).ToArray( );

        // upright, every plane is its face. Turned, 3 of 500 are not, by 39 to 98 degrees: their 1D points are within a
        // quarter of a cell of an edge of the cube, where a face of the grid sees more of the cube than two lines
        if( turn == Turn.Upright ) {
            Assert.LessOrEqual( angles.Last( ), 0.01f, "the largest angle between a plane and its face, in degrees" );
        }
        else {
            Assert.LessOrEqual( angles[( int )( angles.Length * 0.9f )], 0.01f, "the angle between a plane and its face that nine in ten are within, in degrees" );
            Assert.LessOrEqual( angles.Count( ( angle ) => angle > 1.0f ), angles.Length / 100, $"planes more than a degree from their face, of {angles.Length}" );
        }
    }

}

public class GPUOccupancyShapeContourTest : OccupancyShapeContourTest {

    public GPUOccupancyShapeContourTest( ) {
        this.implementation = ImplementationType.GPU;
    }

}
