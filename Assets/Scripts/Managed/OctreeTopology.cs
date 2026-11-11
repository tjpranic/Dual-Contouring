using MaterialIndex = SurfaceExtractor.MaterialIndex;

// Dual Contouring of Hermite Data, section 4: tests on the materials at the corners of octree cells that tell whether
// collapsing a node into a leaf preserves the topology of the contour and of the region each material covers on it.
// Corners are numbered as in the corner layout of the voxels
public static class OctreeTopology {

    // the corners at the ends of each of the twelve edges of a cell
    private static readonly int[,] edgeCorners = {
        { 0, 1 }, { 3, 2 }, { 5, 6 }, { 4, 7 }, // x axis
        { 0, 5 }, { 1, 6 }, { 3, 4 }, { 2, 7 }, // y axis
        { 0, 3 }, { 1, 2 }, { 5, 4 }, { 6, 7 }  // z axis
    };

    // the corners of each of the six faces of a cell, the first two lie diagonally across from each other
    private static readonly int[,] faceCorners = {
        { 0, 2, 1, 3 }, // bottom
        { 4, 6, 5, 7 }, // top
        { 0, 4, 3, 5 }, // minimum x
        { 1, 7, 2, 6 }, // maximum x
        { 0, 6, 1, 5 }, // minimum z
        { 2, 4, 3, 7 }  // maximum z
    };

    // the materials of a corner are a set of flags of which the highest one is drawn, so that is the one that decides
    // which region of the contour the corner belongs to. Empty space is a region of its own
    private static int regionOf( MaterialIndex materialIndex ) {
        return materialIndex == MaterialIndex.Void ? -1 : SurfaceExtractor.findHighestMaterialBit( materialIndex );
    }

    // section 4.2: merge the corners of a cell that are joined by an edge and lie in the same region. The contour
    // inside the cell is a quasi-manifold, a manifold around each region taken by itself, exactly when what is left of
    // the cell is a simplex: a point, a segment, a triangle or a tetrahedron. With nothing but empty and solid corners
    // this is the manifold test of section 4.1, where what is left has to be a single edge
    public static bool isQuasiManifold( MaterialIndex[] corners ) {
        // the corner that stands for the group each corner was merged into
        var groups = new int[] { 0, 1, 2, 3, 4, 5, 6, 7 };

        int groupOf( int corner ) {
            while( groups[corner] != corner ) {
                corner = groups[corner];
            }
            return corner;
        }

        for( var edge = 0; edge < edgeCorners.GetLength( 0 ); ++edge ) {
            var corner = edgeCorners[edge, 0];
            var other  = edgeCorners[edge, 1];

            if( regionOf( corners[corner] ) == regionOf( corners[other] ) ) {
                groups[groupOf( corner )] = groupOf( other );
            }
        }

        // the groups that an edge of the cell still joins directly
        var joined = new bool[groups.Length, groups.Length];

        for( var edge = 0; edge < edgeCorners.GetLength( 0 ); ++edge ) {
            var group = groupOf( edgeCorners[edge, 0] );
            var other = groupOf( edgeCorners[edge, 1] );

            joined[group, other] = true;
            joined[other, group] = true;
        }

        // every point of a simplex is joined to every other. That also rules out a region that comes back as two
        // separate groups, since those are never joined to each other, and more than four groups
        for( var corner = 0; corner < groups.Length; ++corner ) {
            for( var other = corner + 1; other < groups.Length; ++other ) {
                if( groupOf( corner ) == corner && groupOf( other ) == other && !joined[corner, other] ) {
                    return false;
                }
            }
        }

        return true;
    }

    private static MaterialIndex[] materialsOf( SurfaceExtractor.Voxel voxel ) {
        var materials = new MaterialIndex[voxel.corners.Length];

        for( var corner = 0; corner < materials.Length; ++corner ) {
            materials[corner] = voxel.corners[corner].materialIndex;
        }

        return materials;
    }

    // section 4.1: whether a node whose children are all leaves can collapse into a leaf itself without changing the
    // topology of the contour, or of the regions on it
    public static bool collapsePreservesTopology<Voxel>( Octree<Voxel> node ) where Voxel : SurfaceExtractor.Voxel {

        // the first two checks limit collapsing to contours that are quasi-manifolds, in the node as a whole and in
        // each of its children

        if( !isQuasiManifold( materialsOf( node.data ) ) ) {
            return false;
        }

        foreach( var child in node.children ) {
            if( !isQuasiManifold( materialsOf( child.data ) ) ) {
                return false;
            }
        }

        // the third check compares the contour of the children with that of the collapsed node on the edges, the faces
        // and the inside of the node. The corners of the children form a grid of 3 x 3 x 3 points: the child at a
        // corner of the node has its own corners halfway towards each of the other corners of the node

        int regionBetween( int corner, int other ) {
            return regionOf( node.children[corner].data.corners[other].materialIndex );
        }

        // the middle of an edge has to agree with at least one of its two ends
        for( var edge = 0; edge < edgeCorners.GetLength( 0 ); ++edge ) {
            var corner = edgeCorners[edge, 0];
            var other  = edgeCorners[edge, 1];
            var middle = regionBetween( corner, other );

            if( middle != regionBetween( corner, corner ) && middle != regionBetween( other, other ) ) {
                return false;
            }
        }

        // the middle of a face has to agree with at least one of its four corners
        for( var face = 0; face < faceCorners.GetLength( 0 ); ++face ) {
            var middle = regionBetween( faceCorners[face, 0], faceCorners[face, 1] );
            var agrees = false;

            for( var faceCorner = 0; faceCorner < faceCorners.GetLength( 1 ); ++faceCorner ) {
                var corner = faceCorners[face, faceCorner];

                agrees |= middle == regionBetween( corner, corner );
            }

            if( !agrees ) {
                return false;
            }
        }

        // the middle of the node has to agree with at least one of its eight corners
        var center = regionBetween( 0, 7 );

        for( var corner = 0; corner < node.children.Length; ++corner ) {
            if( center == regionBetween( corner, corner ) ) {
                return true;
            }
        }

        return false;
    }

}
