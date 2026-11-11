using MaterialIndex = SurfaceExtractor.MaterialIndex;

// OctreeTopology for Burst code: the same tests, on the materials at the corners of a node and of its children
public static class BurstOctreeTopology {

    // the materials at the corners of a cell
    public unsafe struct CornerMaterials {

        private fixed int materials[8];

        public MaterialIndex this[int corner] {
            readonly get { return ( MaterialIndex )this.materials[corner]; }
            set { this.materials[corner] = ( int )value; }
        }

    }

    // the materials at the corners of the 8 children of a node, child by child. The child at a corner of the node has
    // that corner of the node as its own corner of the same number, and its other corners halfway towards the other
    // corners of the node, which makes these the 3 x 3 x 3 points of the managed test
    public unsafe struct ChildCornerMaterials {

        private fixed int materials[64];

        public MaterialIndex this[int child, int corner] {
            readonly get { return ( MaterialIndex )this.materials[( child * 8 ) + corner]; }
            set { this.materials[( child * 8 ) + corner] = ( int )value; }
        }

    }

    // the corner that stands for the group each corner of a cell was merged into
    private unsafe struct Groups {

        private fixed int corners[8];

        public int this[int corner] {
            readonly get { return this.corners[corner]; }
            set { this.corners[corner] = value; }
        }

        public readonly int groupOf( int corner ) {
            while( this[corner] != corner ) {
                corner = this[corner];
            }
            return corner;
        }

    }

    // the corners of each of the six faces of a cell, the first two lie diagonally across from each other
    private static readonly int[] faceCorners = {
        0, 2, 1, 3, // bottom
        4, 6, 5, 7, // top
        0, 4, 3, 5, // minimum x
        1, 7, 2, 6, // maximum x
        0, 6, 1, 5, // minimum z
        2, 4, 3, 7  // maximum z
    };

    private static int regionOf( MaterialIndex materialIndex ) {
        return materialIndex == MaterialIndex.Void ? -1 : BurstSurfaceExtractor.findHighestMaterialBit( materialIndex );
    }

    // OctreeTopology.isQuasiManifold
    public static bool isQuasiManifold( in CornerMaterials corners ) {
        var groups = new Groups( );
        for( var corner = 0; corner < 8; ++corner ) {
            groups[corner] = corner;
        }

        for( var edge = 0; edge < 12; ++edge ) {
            var corner = BurstTables.lookupEdgeCorner( edge, 0 );
            var other  = BurstTables.lookupEdgeCorner( edge, 1 );

            if( regionOf( corners[corner] ) == regionOf( corners[other] ) ) {
                groups[groups.groupOf( corner )] = groups.groupOf( other );
            }
        }

        // the groups that an edge of the cell still joins directly, a bit for every pair
        var joined = 0UL;

        for( var edge = 0; edge < 12; ++edge ) {
            var group = groups.groupOf( BurstTables.lookupEdgeCorner( edge, 0 ) );
            var other = groups.groupOf( BurstTables.lookupEdgeCorner( edge, 1 ) );

            joined |= 1UL << ( ( group * 8 ) + other );
            joined |= 1UL << ( ( other * 8 ) + group );
        }

        for( var corner = 0; corner < 8; ++corner ) {
            for( var other = corner + 1; other < 8; ++other ) {
                if( groups.groupOf( corner ) == corner && groups.groupOf( other ) == other && ( joined & ( 1UL << ( ( corner * 8 ) + other ) ) ) == 0 ) {
                    return false;
                }
            }
        }

        return true;
    }

    // OctreeTopology.collapsePreservesTopology, for a node whose children are all leaves
    public static bool collapsePreservesTopology( in ChildCornerMaterials children ) {
        var node = new CornerMaterials( );
        for( var corner = 0; corner < 8; ++corner ) {
            node[corner] = children[corner, corner];
        }

        if( !isQuasiManifold( node ) ) {
            return false;
        }

        for( var child = 0; child < 8; ++child ) {
            var corners = new CornerMaterials( );
            for( var corner = 0; corner < 8; ++corner ) {
                corners[corner] = children[child, corner];
            }

            if( !isQuasiManifold( corners ) ) {
                return false;
            }
        }

        // the middle of an edge has to agree with at least one of its two ends
        for( var edge = 0; edge < 12; ++edge ) {
            var corner = BurstTables.lookupEdgeCorner( edge, 0 );
            var other  = BurstTables.lookupEdgeCorner( edge, 1 );
            var middle = regionOf( children[corner, other] );

            if( middle != regionOf( children[corner, corner] ) && middle != regionOf( children[other, other] ) ) {
                return false;
            }
        }

        // the middle of a face has to agree with at least one of its four corners
        for( var face = 0; face < 6; ++face ) {
            var middle = regionOf( children[faceCorners[( face * 4 ) + 0], faceCorners[( face * 4 ) + 1]] );
            var agrees = false;

            for( var faceCorner = 0; faceCorner < 4; ++faceCorner ) {
                var corner = faceCorners[( face * 4 ) + faceCorner];

                agrees |= middle == regionOf( children[corner, corner] );
            }

            if( !agrees ) {
                return false;
            }
        }

        // the middle of the node has to agree with at least one of its eight corners
        var center = regionOf( children[0, 7] );

        for( var corner = 0; corner < 8; ++corner ) {
            if( center == regionOf( children[corner, corner] ) ) {
                return true;
            }
        }

        return false;
    }

}
