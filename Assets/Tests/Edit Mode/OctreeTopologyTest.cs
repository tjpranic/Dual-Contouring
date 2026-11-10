using NUnit.Framework;

using MaterialIndex = SurfaceExtractor.MaterialIndex;

// Dual Contouring of Hermite Data, section 4: merging the corners of a cell that share an edge and a material leaves
// a simplex exactly when the contour inside the cell is a (quasi-)manifold
public class OctreeTopologyTest {

    private const MaterialIndex _ = MaterialIndex.Void;
    private const MaterialIndex A = MaterialIndex.Material0;
    private const MaterialIndex B = MaterialIndex.Material1;
    private const MaterialIndex C = MaterialIndex.Material2;
    private const MaterialIndex D = MaterialIndex.Material3;
    private const MaterialIndex E = MaterialIndex.Material4;

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
    private static bool isQuasiManifold( params MaterialIndex[] corners ) {
        return OctreeTopology.isQuasiManifold( corners );
    }

    [Test]
    public void cellWithoutContourIsManifold( ) {
        Assert.IsTrue( isQuasiManifold( _, _, _, _, _, _, _, _ ), "empty" );
        Assert.IsTrue( isQuasiManifold( A, A, A, A, A, A, A, A ), "solid" );
    }

    [Test]
    public void cellCutByOneSheetIsManifold( ) {
        Assert.IsTrue( isQuasiManifold( A, _, _, _, _, _, _, _ ), "one solid corner" );
        Assert.IsTrue( isQuasiManifold( A, A, _, _, _, _, _, _ ), "two solid corners along an edge" );
        Assert.IsTrue( isQuasiManifold( A, A, A, A, _, _, _, _ ), "solid bottom face" );
        Assert.IsTrue( isQuasiManifold( A, A, A, A, A, A, A, _ ), "one empty corner" );
    }

    // the ambiguous sign configurations of marching cubes: the solid corners, or the empty ones, fall apart into pieces
    // that no edge of the cell connects
    [Test]
    public void cellCutByTwoSheetsIsNotManifold( ) {
        Assert.IsFalse( isQuasiManifold( A, _, A, _, _, _, _, _ ), "two solid corners diagonally across a face" );
        Assert.IsFalse( isQuasiManifold( A, _, _, _, _, _, _, A ), "two solid corners diagonally across the cell" );
        Assert.IsFalse( isQuasiManifold( _, A, _, A, A, A, A, A ), "two empty corners diagonally across a face" );
    }

    // the highest material of a corner is the one that is drawn, corners that only differ below it are one region
    [Test]
    public void regionIsDecidedByHighestMaterial( ) {
        Assert.IsTrue(  isQuasiManifold( B, A | B, _, _, _, _, _, _ ), "the same region along an edge" );
        Assert.IsFalse( isQuasiManifold( B, _, A | B, _, _, _, _, _ ), "the same region diagonally across a face" );
    }

    [Test]
    public void cellWhereMaterialsMeetIsQuasiManifold( ) {
        Assert.IsTrue( isQuasiManifold( A, B, _, _, _, _, _, _ ), "two materials and empty space that all touch: a triangle" );
        Assert.IsTrue( isQuasiManifold( A, A, B, B, B, A, A, B ), "two materials that fill the cell: a segment" );

        // corner 0 | corners 1, 6 | corners 5, 4 | corners 3, 2, 7: each group touches the other three
        Assert.IsTrue( isQuasiManifold( A, B, D, D, C, C, B, D ), "four materials that all touch: a tetrahedron" );
    }

    [Test]
    public void cellWhereMaterialsDoNotAllTouchIsNotQuasiManifold( ) {
        Assert.IsFalse( isQuasiManifold( A, _, B, _, _, _, _, _ ), "two materials diagonally across a face, joined only through empty space" );
        // corners 0, 1 | corners 3, 2 | corners 4, 7 | corners 5, 6: the four edges along x, each touches two of the others
        Assert.IsFalse( isQuasiManifold( A, A, B, B, C, D, D, C ), "four materials in a ring, opposite ones do not touch" );
        Assert.IsFalse( isQuasiManifold( A, B, C, D, E, _, _, _ ), "five materials and empty space" );
    }

}
