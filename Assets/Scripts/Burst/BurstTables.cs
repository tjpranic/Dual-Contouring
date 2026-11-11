using MaterialIndex = SurfaceExtractor.MaterialIndex;

// the lookup tables of the managed implementations as constant data, which Burst code can read without a native
// container to look after. Corners and edges are numbered as in the corner and edge layouts of the voxels, axes are 0
// for x, 1 for y and 2 for z, and children of an octree node are numbered as in the child layout of the octrees
public static class BurstTables {

    // where each corner of a cell is, in units of the cell from its minimum. Child nodes sit at the same places
    private static readonly int[] cornerOffsets = {
        0, 0, 0,
        1, 0, 0,
        1, 0, 1,
        0, 0, 1,
        0, 1, 1,
        0, 1, 0,
        1, 1, 0,
        1, 1, 1
    };

    // the corners at the ends of each edge, from the minimum to the maximum of the axis it runs along
    private static readonly int[] edgeCorners = {
        0, 1,  3, 2,  5, 6,  4, 7, // x axis
        0, 5,  1, 6,  3, 4,  2, 7, // y axis
        0, 3,  1, 2,  5, 4,  6, 7  // z axis
    };

    // the edges around each of the six faces of a cell
    private static readonly int[] faceEdges = {
        0, 1,  8,  9, // minimum y
        0, 2,  4,  5, // minimum z
        1, 3,  6,  7, // maximum z
        4, 6,  8, 10, // minimum x
        5, 7,  9, 11, // maximum x
        2, 3, 10, 11  // maximum y
    };

    public static int lookupCornerOffset( int corner, int axis ) {
        return cornerOffsets[( corner * 3 ) + axis];
    }

    public static int lookupEdgeCorner( int edge, int end ) {
        return edgeCorners[( edge * 2 ) + end];
    }

    public static int lookupFaceEdge( int face, int index ) {
        return faceEdges[( face * 4 ) + index];
    }

    // whether two edges of a cell end in the same corner
    public static bool edgesShareCorner( int edge, int other ) {
        return lookupEdgeCorner( edge, 0 ) == lookupEdgeCorner( other, 0 ) || lookupEdgeCorner( edge, 0 ) == lookupEdgeCorner( other, 1 ) ||
               lookupEdgeCorner( edge, 1 ) == lookupEdgeCorner( other, 0 ) || lookupEdgeCorner( edge, 1 ) == lookupEdgeCorner( other, 1 );
    }

    // the corners of a cell as a set of bits, bit n for corner n, of the corners that are solid
    public static int solidCornersOf(
        MaterialIndex corner0, MaterialIndex corner1, MaterialIndex corner2, MaterialIndex corner3,
        MaterialIndex corner4, MaterialIndex corner5, MaterialIndex corner6, MaterialIndex corner7
    ) {
        return ( corner0 != MaterialIndex.Void ? 1 << 0 : 0 ) |
               ( corner1 != MaterialIndex.Void ? 1 << 1 : 0 ) |
               ( corner2 != MaterialIndex.Void ? 1 << 2 : 0 ) |
               ( corner3 != MaterialIndex.Void ? 1 << 3 : 0 ) |
               ( corner4 != MaterialIndex.Void ? 1 << 4 : 0 ) |
               ( corner5 != MaterialIndex.Void ? 1 << 5 : 0 ) |
               ( corner6 != MaterialIndex.Void ? 1 << 6 : 0 ) |
               ( corner7 != MaterialIndex.Void ? 1 << 7 : 0 );
    }

    // DualMarchingCubes.lookupVertexEdges as two tables, generated from it: for each of the 256 ways the corners of a
    // cell can be solid or empty, the vertex each of the 12 edges belongs to, or -1 for an edge the contour does not
    // intersect, and how many vertices the cell has. The vertices are numbered in the order the managed table lists them
    private static readonly sbyte[] edgeVertices = {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0
         0, -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, // 1
         0, -1, -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, // 2
        -1, -1, -1, -1,  0,  0, -1, -1,  0,  0, -1, -1, // 3
        -1,  0, -1, -1, -1, -1, -1,  0, -1,  0, -1, -1, // 4
         0,  1, -1, -1,  0, -1, -1,  1,  0,  1, -1, -1, // 5
         0,  0, -1, -1, -1,  0, -1,  0, -1, -1, -1, -1, // 6
        -1,  0, -1, -1,  0,  0, -1,  0,  0, -1, -1, -1, // 7
        -1,  0, -1, -1, -1, -1,  0, -1,  0, -1, -1, -1, // 8
         0,  0, -1, -1,  0, -1,  0, -1, -1, -1, -1, -1, // 9
         1,  0, -1, -1, -1,  1,  0, -1,  0,  1, -1, -1, // 10
        -1,  0, -1, -1,  0,  0,  0, -1, -1,  0, -1, -1, // 11
        -1, -1, -1, -1, -1, -1,  0,  0,  0,  0, -1, -1, // 12
         0, -1, -1, -1,  0, -1,  0,  0, -1,  0, -1, -1, // 13
         0, -1, -1, -1, -1,  0,  0,  0,  0, -1, -1, -1, // 14
        -1, -1, -1, -1,  0,  0,  0,  0, -1, -1, -1, -1, // 15
        -1, -1, -1,  0, -1, -1,  0, -1, -1, -1,  0, -1, // 16
         0, -1, -1,  1,  0, -1,  1, -1,  0, -1,  1, -1, // 17
         0, -1, -1,  1, -1,  0,  1, -1, -1,  0,  1, -1, // 18
        -1, -1, -1,  1,  0,  0,  1, -1,  0,  0,  1, -1, // 19
        -1,  0, -1,  1, -1, -1,  1,  0, -1,  0,  1, -1, // 20
         0,  1, -1,  2,  0, -1,  2,  1,  0,  1,  2, -1, // 21
         0,  0, -1,  1, -1,  0,  1,  0, -1, -1,  1, -1, // 22
        -1,  0, -1,  1,  0,  0,  1,  0,  0, -1,  1, -1, // 23
        -1,  0, -1,  0, -1, -1, -1, -1,  0, -1,  0, -1, // 24
         0,  0, -1,  0,  0, -1, -1, -1, -1, -1,  0, -1, // 25
         0,  1, -1,  1, -1,  0, -1, -1,  1,  0,  1, -1, // 26
        -1,  0, -1,  0,  0,  0, -1, -1, -1,  0,  0, -1, // 27
        -1, -1, -1,  0, -1, -1, -1,  0,  0,  0,  0, -1, // 28
         0, -1, -1,  0,  0, -1, -1,  0, -1,  0,  0, -1, // 29
         0, -1, -1,  0, -1,  0, -1,  0,  0, -1,  0, -1, // 30
        -1, -1, -1,  0,  0,  0, -1,  0, -1, -1,  0, -1, // 31
        -1, -1,  0, -1,  0, -1, -1, -1, -1, -1,  0, -1, // 32
         0, -1,  0, -1, -1, -1, -1, -1,  0, -1,  0, -1, // 33
         0, -1,  1, -1,  1,  0, -1, -1, -1,  0,  1, -1, // 34
        -1, -1,  0, -1, -1,  0, -1, -1,  0,  0,  0, -1, // 35
        -1,  0,  1, -1,  1, -1, -1,  0, -1,  0,  1, -1, // 36
         1,  0,  1, -1, -1, -1, -1,  0,  1,  0,  1, -1, // 37
         0,  0,  1, -1,  1,  0, -1,  0, -1, -1,  1, -1, // 38
        -1,  0,  0, -1, -1,  0, -1,  0,  0, -1,  0, -1, // 39
        -1,  0,  1, -1,  1, -1,  0, -1,  0, -1,  1, -1, // 40
         0,  0,  0, -1, -1, -1,  0, -1, -1, -1,  0, -1, // 41
         1,  0,  2, -1,  2,  1,  0, -1,  0,  1,  2, -1, // 42
        -1,  0,  0, -1, -1,  0,  0, -1, -1,  0,  0, -1, // 43
        -1, -1,  1, -1,  1, -1,  0,  0,  0,  0,  1, -1, // 44
         0, -1,  0, -1, -1, -1,  0,  0, -1,  0,  0, -1, // 45
         0, -1,  1, -1,  1,  0,  0,  0,  0, -1,  1, -1, // 46
        -1, -1,  0, -1, -1,  0,  0,  0, -1, -1,  0, -1, // 47
        -1, -1,  0,  0,  0, -1,  0, -1, -1, -1, -1, -1, // 48
         0, -1,  0,  0, -1, -1,  0, -1,  0, -1, -1, -1, // 49
         1, -1,  0,  0,  0,  1,  0, -1, -1,  1, -1, -1, // 50
        -1, -1,  0,  0, -1,  0,  0, -1,  0,  0, -1, -1, // 51
        -1,  1,  0,  0,  0, -1,  0,  1, -1,  1, -1, -1, // 52
         0,  1,  0,  0, -1, -1,  0,  1,  0,  1, -1, -1, // 53
         1,  1,  0,  0,  0,  1,  0,  1, -1, -1, -1, -1, // 54
        -1,  0,  0,  0, -1,  0,  0,  0,  0, -1, -1, -1, // 55
        -1,  0,  0,  0,  0, -1, -1, -1,  0, -1, -1, -1, // 56
         0,  0,  0,  0, -1, -1, -1, -1, -1, -1, -1, -1, // 57
         1,  0,  0,  0,  0,  1, -1, -1,  0,  1, -1, -1, // 58
        -1,  0,  0,  0, -1,  0, -1, -1, -1,  0, -1, -1, // 59
        -1, -1,  0,  0,  0, -1, -1,  0,  0,  0, -1, -1, // 60
         0, -1,  0,  0, -1, -1, -1,  0, -1,  0, -1, -1, // 61
         0, -1,  0,  0,  0,  0, -1,  0,  0, -1, -1, -1, // 62
        -1, -1,  0,  0, -1,  0, -1,  0, -1, -1, -1, -1, // 63
        -1, -1,  0, -1, -1,  0, -1, -1, -1, -1, -1,  0, // 64
         0, -1,  1, -1,  0,  1, -1, -1,  0, -1, -1,  1, // 65
         0, -1,  0, -1, -1, -1, -1, -1, -1,  0, -1,  0, // 66
        -1, -1,  0, -1,  0, -1, -1, -1,  0,  0, -1,  0, // 67
        -1,  0,  1, -1, -1,  1, -1,  0, -1,  0, -1,  1, // 68
         0,  1,  2, -1,  0,  2, -1,  1,  0,  1, -1,  2, // 69
         0,  0,  0, -1, -1, -1, -1,  0, -1, -1, -1,  0, // 70
        -1,  0,  0, -1,  0, -1, -1,  0,  0, -1, -1,  0, // 71
        -1,  0,  1, -1, -1,  1,  0, -1,  0, -1, -1,  1, // 72
         0,  0,  1, -1,  0,  1,  0, -1, -1, -1, -1,  1, // 73
         1,  0,  1, -1, -1, -1,  0, -1,  0,  1, -1,  1, // 74
        -1,  0,  0, -1,  0, -1,  0, -1, -1,  0, -1,  0, // 75
        -1, -1,  1, -1, -1,  1,  0,  0,  0,  0, -1,  1, // 76
         0, -1,  1, -1,  0,  1,  0,  0, -1,  0, -1,  1, // 77
         0, -1,  0, -1, -1, -1,  0,  0,  0, -1, -1,  0, // 78
        -1, -1,  0, -1,  0, -1,  0,  0, -1, -1, -1,  0, // 79
        -1, -1,  1,  0, -1,  1,  0, -1, -1, -1,  0,  1, // 80
         0, -1,  2,  1,  0,  2,  1, -1,  0, -1,  1,  2, // 81
         1, -1,  1,  0, -1, -1,  0, -1, -1,  1,  0,  1, // 82
        -1, -1,  1,  0,  1, -1,  0, -1,  1,  1,  0,  1, // 83
        -1,  0,  2,  1, -1,  2,  1,  0, -1,  0,  1,  2, // 84
         0,  1,  3,  2,  0,  3,  2,  1,  0,  1,  2,  3, // 85
         1,  1,  1,  0, -1, -1,  0,  1, -1, -1,  0,  1, // 86
        -1,  0,  0,  1,  0, -1,  1,  0,  0, -1,  1,  0, // 87
        -1,  0,  1,  0, -1,  1, -1, -1,  0, -1,  0,  1, // 88
         0,  0,  1,  0,  0,  1, -1, -1, -1, -1,  0,  1, // 89
         1,  0,  1,  0, -1, -1, -1, -1,  0,  1,  0,  1, // 90
        -1,  0,  0,  0,  0, -1, -1, -1, -1,  0,  0,  0, // 91
        -1, -1,  1,  0, -1,  1, -1,  0,  0,  0,  0,  1, // 92
         0, -1,  1,  0,  0,  1, -1,  0, -1,  0,  0,  1, // 93
         0, -1,  0,  0, -1, -1, -1,  0,  0, -1,  0,  0, // 94
        -1, -1,  0,  0,  0, -1, -1,  0, -1, -1,  0,  0, // 95
        -1, -1, -1, -1,  0,  0, -1, -1, -1, -1,  0,  0, // 96
         0, -1, -1, -1, -1,  0, -1, -1,  0, -1,  0,  0, // 97
         0, -1, -1, -1,  0, -1, -1, -1, -1,  0,  0,  0, // 98
        -1, -1, -1, -1, -1, -1, -1, -1,  0,  0,  0,  0, // 99
        -1,  0, -1, -1,  1,  1, -1,  0, -1,  0,  1,  1, // 100
         1,  0, -1, -1, -1,  1, -1,  0,  1,  0,  1,  1, // 101
         0,  0, -1, -1,  0, -1, -1,  0, -1, -1,  0,  0, // 102
        -1,  0, -1, -1, -1, -1, -1,  0,  0, -1,  0,  0, // 103
        -1,  0, -1, -1,  1,  1,  0, -1,  0, -1,  1,  1, // 104
         0,  0, -1, -1, -1,  0,  0, -1, -1, -1,  0,  0, // 105
         1,  0, -1, -1,  1, -1,  0, -1,  0,  1,  1,  1, // 106
        -1,  0, -1, -1, -1, -1,  0, -1, -1,  0,  0,  0, // 107
        -1, -1, -1, -1,  1,  1,  0,  0,  0,  0,  1,  1, // 108
         0, -1, -1, -1, -1,  0,  0,  0, -1,  0,  0,  0, // 109
         0, -1, -1, -1,  0, -1,  0,  0,  0, -1,  0,  0, // 110
        -1, -1, -1, -1, -1, -1,  0,  0, -1, -1,  0,  0, // 111
        -1, -1, -1,  0,  0,  0,  0, -1, -1, -1, -1,  0, // 112
         0, -1, -1,  0, -1,  0,  0, -1,  0, -1, -1,  0, // 113
         0, -1, -1,  0,  0, -1,  0, -1, -1,  0, -1,  0, // 114
        -1, -1, -1,  0, -1, -1,  0, -1,  0,  0, -1,  0, // 115
        -1,  0, -1,  1,  1,  1,  1,  0, -1,  0, -1,  1, // 116
         1,  0, -1,  1, -1,  1,  1,  0,  1,  0, -1,  1, // 117
         0,  0, -1,  0,  0, -1,  0,  0, -1, -1, -1,  0, // 118
        -1,  0, -1,  0, -1, -1,  0,  0,  0, -1, -1,  0, // 119
        -1,  0, -1,  0,  0,  0, -1, -1,  0, -1, -1,  0, // 120
         0,  0, -1,  0, -1,  0, -1, -1, -1, -1, -1,  0, // 121
         0,  0, -1,  0,  0, -1, -1, -1,  0,  0, -1,  0, // 122
        -1,  0, -1,  0, -1, -1, -1, -1, -1,  0, -1,  0, // 123
        -1, -1, -1,  0,  0,  0, -1,  0,  0,  0, -1,  0, // 124
         0, -1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, // 125
         0, -1, -1,  1,  0, -1, -1,  1,  0, -1, -1,  1, // 126
        -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1,  0, // 127
        -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1,  0, // 128
         0, -1, -1,  1,  0, -1, -1,  1,  0, -1, -1,  1, // 129
         0, -1, -1,  1, -1,  0, -1,  1, -1,  0, -1,  1, // 130
        -1, -1, -1,  1,  0,  0, -1,  1,  0,  0, -1,  1, // 131
        -1,  0, -1,  0, -1, -1, -1, -1, -1,  0, -1,  0, // 132
         0,  1, -1,  1,  0, -1, -1, -1,  0,  1, -1,  1, // 133
         0,  0, -1,  0, -1,  0, -1, -1, -1, -1, -1,  0, // 134
        -1,  0, -1,  0,  0,  0, -1, -1,  0, -1, -1,  0, // 135
        -1,  0, -1,  1, -1, -1,  0,  1,  0, -1, -1,  1, // 136
         0,  0, -1,  1,  0, -1,  0,  1, -1, -1, -1,  1, // 137
         1,  0, -1,  2, -1,  1,  0,  2,  0,  1, -1,  2, // 138
        -1,  0, -1,  1,  0,  0,  0,  1, -1,  0, -1,  1, // 139
        -1, -1, -1,  0, -1, -1,  0, -1,  0,  0, -1,  0, // 140
         0, -1, -1,  0,  0, -1,  0, -1, -1,  0, -1,  0, // 141
         0, -1, -1,  0, -1,  0,  0, -1,  0, -1, -1,  0, // 142
        -1, -1, -1,  0,  0,  0,  0, -1, -1, -1, -1,  0, // 143
        -1, -1, -1, -1, -1, -1,  0,  0, -1, -1,  0,  0, // 144
         0, -1, -1, -1,  0, -1,  1,  1,  0, -1,  1,  1, // 145
         0, -1, -1, -1, -1,  0,  1,  1, -1,  0,  1,  1, // 146
        -1, -1, -1, -1,  0,  0,  1,  1,  0,  0,  1,  1, // 147
        -1,  0, -1, -1, -1, -1,  0, -1, -1,  0,  0,  0, // 148
         0,  1, -1, -1,  0, -1,  1, -1,  0,  1,  1,  1, // 149
         0,  0, -1, -1, -1,  0,  0, -1, -1, -1,  0,  0, // 150
        -1,  0, -1, -1,  0,  0,  0, -1,  0, -1,  0,  0, // 151
        -1,  0, -1, -1, -1, -1, -1,  0,  0, -1,  0,  0, // 152
         0,  0, -1, -1,  0, -1, -1,  0, -1, -1,  0,  0, // 153
         0,  1, -1, -1, -1,  0, -1,  1,  1,  0,  1,  1, // 154
        -1,  0, -1, -1,  0,  0, -1,  0, -1,  0,  0,  0, // 155
        -1, -1, -1, -1, -1, -1, -1, -1,  0,  0,  0,  0, // 156
         0, -1, -1, -1,  0, -1, -1, -1, -1,  0,  0,  0, // 157
         0, -1, -1, -1, -1,  0, -1, -1,  0, -1,  0,  0, // 158
        -1, -1, -1, -1,  0,  0, -1, -1, -1, -1,  0,  0, // 159
        -1, -1,  0,  1,  0, -1, -1,  1, -1, -1,  0,  1, // 160
         0, -1,  0,  1, -1, -1, -1,  1,  0, -1,  0,  1, // 161
         0, -1,  1,  2,  1,  0, -1,  2, -1,  0,  1,  2, // 162
        -1, -1,  0,  1, -1,  0, -1,  1,  0,  0,  0,  1, // 163
        -1,  1,  0,  1,  0, -1, -1, -1, -1,  1,  0,  1, // 164
         0,  1,  0,  1, -1, -1, -1, -1,  0,  1,  0,  1, // 165
         1,  1,  0,  1,  0,  1, -1, -1, -1, -1,  0,  1, // 166
        -1,  0,  0,  0, -1,  0, -1, -1,  0, -1,  0,  0, // 167
        -1,  0,  1,  2,  1, -1,  0,  2,  0, -1,  1,  2, // 168
         0,  0,  0,  1, -1, -1,  0,  1, -1, -1,  0,  1, // 169
         1,  0,  2,  3,  2,  1,  0,  3,  0,  1,  2,  3, // 170
        -1,  0,  0,  1, -1,  0,  0,  1, -1,  0,  0,  1, // 171
        -1, -1,  0,  1,  0, -1,  1, -1,  1,  1,  0,  1, // 172
         0, -1,  0,  0, -1, -1,  0, -1, -1,  0,  0,  0, // 173
         1, -1,  0,  1,  0,  1,  1, -1,  1, -1,  0,  1, // 174
        -1, -1,  0,  0, -1,  0,  0, -1, -1, -1,  0,  0, // 175
        -1, -1,  0, -1,  0, -1,  0,  0, -1, -1, -1,  0, // 176
         0, -1,  0, -1, -1, -1,  0,  0,  0, -1, -1,  0, // 177
         1, -1,  0, -1,  0,  1,  0,  0, -1,  1, -1,  0, // 178
        -1, -1,  0, -1, -1,  0,  0,  0,  0,  0, -1,  0, // 179
        -1,  0,  0, -1,  0, -1,  0, -1, -1,  0, -1,  0, // 180
         0,  0,  0, -1, -1, -1,  0, -1,  0,  0, -1,  0, // 181
         0,  0,  0, -1,  0,  0,  0, -1, -1, -1, -1,  0, // 182
        -1,  0,  1, -1, -1,  1,  0, -1,  0, -1, -1,  1, // 183
        -1,  0,  0, -1,  0, -1, -1,  0,  0, -1, -1,  0, // 184
         0,  0,  0, -1, -1, -1, -1,  0, -1, -1, -1,  0, // 185
         1,  0,  0, -1,  0,  1, -1,  0,  0,  1, -1,  0, // 186
        -1,  0,  0, -1, -1,  0, -1,  0, -1,  0, -1,  0, // 187
        -1, -1,  0, -1,  0, -1, -1, -1,  0,  0, -1,  0, // 188
         0, -1,  0, -1, -1, -1, -1, -1, -1,  0, -1,  0, // 189
         0, -1,  0, -1,  0,  0, -1, -1,  0, -1, -1,  0, // 190
        -1, -1,  0, -1, -1,  0, -1, -1, -1, -1, -1,  0, // 191
        -1, -1,  0,  0, -1,  0, -1,  0, -1, -1, -1, -1, // 192
         1, -1,  0,  0,  1,  0, -1,  0,  1, -1, -1, -1, // 193
         0, -1,  0,  0, -1, -1, -1,  0, -1,  0, -1, -1, // 194
        -1, -1,  0,  0,  0, -1, -1,  0,  0,  0, -1, -1, // 195
        -1,  0,  0,  0, -1,  0, -1, -1, -1,  0, -1, -1, // 196
         0,  1,  1,  1,  0,  1, -1, -1,  0,  1, -1, -1, // 197
         0,  0,  0,  0, -1, -1, -1, -1, -1, -1, -1, -1, // 198
        -1,  0,  0,  0,  0, -1, -1, -1,  0, -1, -1, -1, // 199
        -1,  1,  0,  0, -1,  0,  1,  0,  1, -1, -1, -1, // 200
         0,  0,  1,  1,  0,  1,  0,  1, -1, -1, -1, -1, // 201
         0,  1,  0,  0, -1, -1,  1,  0,  1,  0, -1, -1, // 202
        -1,  0,  0,  0,  0, -1,  0,  0, -1,  0, -1, -1, // 203
        -1, -1,  0,  0, -1,  0,  0, -1,  0,  0, -1, -1, // 204
         0, -1,  0,  0,  0,  0,  0, -1, -1,  0, -1, -1, // 205
         0, -1,  0,  0, -1, -1,  0, -1,  0, -1, -1, -1, // 206
        -1, -1,  0,  0,  0, -1,  0, -1, -1, -1, -1, -1, // 207
        -1, -1,  0, -1, -1,  0,  0,  0, -1, -1,  0, -1, // 208
         1, -1,  0, -1,  1,  0,  0,  0,  1, -1,  0, -1, // 209
         0, -1,  0, -1, -1, -1,  0,  0, -1,  0,  0, -1, // 210
        -1, -1,  0, -1,  0, -1,  0,  0,  0,  0,  0, -1, // 211
        -1,  0,  0, -1, -1,  0,  0, -1, -1,  0,  0, -1, // 212
         0,  1,  1, -1,  0,  1,  1, -1,  0,  1,  1, -1, // 213
         0,  0,  0, -1, -1, -1,  0, -1, -1, -1,  0, -1, // 214
        -1,  0,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, // 215
        -1,  0,  0, -1, -1,  0, -1,  0,  0, -1,  0, -1, // 216
         0,  0,  0, -1,  0,  0, -1,  0, -1, -1,  0, -1, // 217
         0,  0,  0, -1, -1, -1, -1,  0,  0,  0,  0, -1, // 218
        -1,  0,  1, -1,  1, -1, -1,  0, -1,  0,  1, -1, // 219
        -1, -1,  0, -1, -1,  0, -1, -1,  0,  0,  0, -1, // 220
         0, -1,  0, -1,  0,  0, -1, -1, -1,  0,  0, -1, // 221
         0, -1,  0, -1, -1, -1, -1, -1,  0, -1,  0, -1, // 222
        -1, -1,  0, -1,  0, -1, -1, -1, -1, -1,  0, -1, // 223
        -1, -1, -1,  0,  0,  0, -1,  0, -1, -1,  0, -1, // 224
         0, -1, -1,  0, -1,  0, -1,  0,  0, -1,  0, -1, // 225
         0, -1, -1,  0,  0, -1, -1,  0, -1,  0,  0, -1, // 226
        -1, -1, -1,  0, -1, -1, -1,  0,  0,  0,  0, -1, // 227
        -1,  0, -1,  0,  0,  0, -1, -1, -1,  0,  0, -1, // 228
         0,  0, -1,  0, -1,  0, -1, -1,  0,  0,  0, -1, // 229
         0,  0, -1,  0,  0, -1, -1, -1, -1, -1,  0, -1, // 230
        -1,  0, -1,  0, -1, -1, -1, -1,  0, -1,  0, -1, // 231
        -1,  1, -1,  0,  0,  0,  1,  0,  1, -1,  0, -1, // 232
         0,  0, -1,  0, -1,  0,  0,  0, -1, -1,  0, -1, // 233
         1,  0, -1,  1,  1, -1,  0,  1,  0,  1,  1, -1, // 234
        -1,  0, -1,  0, -1, -1,  0,  0, -1,  0,  0, -1, // 235
        -1, -1, -1,  0,  0,  0,  0, -1,  0,  0,  0, -1, // 236
         0, -1, -1,  1, -1,  0,  1, -1, -1,  0,  1, -1, // 237
         0, -1, -1,  0,  0, -1,  0, -1,  0, -1,  0, -1, // 238
        -1, -1, -1,  0, -1, -1,  0, -1, -1, -1,  0, -1, // 239
        -1, -1, -1, -1,  0,  0,  0,  0, -1, -1, -1, -1, // 240
         0, -1, -1, -1, -1,  0,  0,  0,  0, -1, -1, -1, // 241
         0, -1, -1, -1,  0, -1,  0,  0, -1,  0, -1, -1, // 242
        -1, -1, -1, -1, -1, -1,  0,  0,  0,  0, -1, -1, // 243
        -1,  0, -1, -1,  0,  0,  0, -1, -1,  0, -1, -1, // 244
         0,  0, -1, -1, -1,  0,  0, -1,  0,  0, -1, -1, // 245
         0,  0, -1, -1,  0, -1,  0, -1, -1, -1, -1, -1, // 246
        -1,  0, -1, -1, -1, -1,  0, -1,  0, -1, -1, -1, // 247
        -1,  0, -1, -1,  0,  0, -1,  0,  0, -1, -1, -1, // 248
         0,  0, -1, -1, -1,  0, -1,  0, -1, -1, -1, -1, // 249
         0,  0, -1, -1,  0, -1, -1,  0,  0,  0, -1, -1, // 250
        -1,  0, -1, -1, -1, -1, -1,  0, -1,  0, -1, -1, // 251
        -1, -1, -1, -1,  0,  0, -1, -1,  0,  0, -1, -1, // 252
         0, -1, -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, // 253
         0, -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, // 254
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1  // 255
    };

    private static readonly byte[] vertexCounts = {
        0, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1,
        1, 1, 2, 1, 2, 2, 2, 1, 2, 1, 3, 1, 2, 1, 2, 1, 1, 1, 2, 1, 2, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1,
        1, 2, 1, 1, 2, 3, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1, 2, 3, 2, 2, 3, 4, 2, 2, 2, 2, 2, 1, 2, 2, 1, 1,
        1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1,
        1, 2, 2, 2, 1, 2, 1, 1, 2, 2, 3, 2, 1, 1, 1, 1, 1, 2, 2, 2, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
        2, 2, 3, 2, 2, 2, 2, 1, 3, 2, 4, 2, 2, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1,
        1, 2, 1, 1, 1, 2, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0
    };

    public static int lookupVertexCount( int solidCorners ) {
        return vertexCounts[solidCorners];
    }

    public static int lookupEdgeVertex( int solidCorners, int edge ) {
        return edgeVertices[( solidCorners * 12 ) + edge];
    }

    // OctreeContouringTables for the root position, the only one the implementations use, as the children each node
    // passes on. Wherever the managed tables take a child of a node that node is internal, otherwise the node stands in
    // for itself

    // lookupEdgeNodesWithinNode: for each axis, the 2 groups of 4 children around the edges inside a node
    private static readonly int[] edgeNodesWithinNode = {
        0, 3, 5, 4,  1, 2, 6, 7, // x axis
        0, 1, 3, 2,  5, 6, 4, 7, // y axis
        0, 5, 1, 6,  3, 4, 2, 7  // z axis
    };

    // lookupFacePairsWithinNode: for each axis, the 4 pairs of children on either side of the faces inside a node
    private static readonly int[] facePairsWithinNode = {
        0, 1,  3, 2,  5, 6,  4, 7, // x axis
        0, 5,  1, 6,  3, 4,  2, 7, // y axis
        0, 3,  1, 2,  5, 4,  6, 7  // z axis
    };

    // lookupFacePairsWithinFacePairs: for each axis, the 4 pairs of children that the face between two nodes is made
    // of, the first child of the first node and the second of the second
    private static readonly int[] facePairsWithinFacePairs = {
        1, 0,  2, 3,  6, 5,  7, 4, // x axis
        5, 0,  6, 1,  4, 3,  7, 2, // y axis
        3, 0,  2, 1,  4, 5,  7, 6  // z axis
    };

    // lookupEdgeNodesWithinFacePairs: for each axis, the 4 edges inside the face between two nodes, as 4 children each,
    // which of the two nodes each child is of, and the axis the edge runs along
    private static readonly int[] edgeNodesWithinFacePairsChildren = {
        1, 0, 2, 3,  6, 5, 7, 4,  1, 6, 0, 5,  2, 7, 3, 4, // x axis
        5, 4, 0, 3,  6, 7, 1, 2,  5, 0, 6, 1,  4, 3, 7, 2, // y axis
        3, 0, 4, 5,  2, 1, 7, 6,  3, 2, 0, 1,  4, 7, 5, 6  // z axis
    };

    private static readonly int[] edgeNodesWithinFacePairsNodes = {
        0, 1, 0, 1,  0, 1, 0, 1,  0, 0, 1, 1,  0, 0, 1, 1, // x axis
        0, 0, 1, 1,  0, 0, 1, 1,  0, 1, 0, 1,  0, 1, 0, 1, // y axis
        0, 1, 0, 1,  0, 1, 0, 1,  0, 0, 1, 1,  0, 0, 1, 1  // z axis
    };

    private static readonly int[] edgeNodesWithinFacePairsAxes = {
        1, 1, 2, 2, // x axis
        0, 0, 2, 2, // y axis
        0, 0, 1, 1  // z axis
    };

    // lookupEdgeNodesWithinEdgeNodes: for each axis, the 2 groups of 4 children along the edge that 4 nodes have in
    // common, the n-th child of the n-th node
    private static readonly int[] edgeNodesWithinEdgeNodes = {
        4, 5, 3, 0,  7, 6, 2, 1, // x axis
        2, 3, 1, 0,  7, 4, 6, 5, // y axis
        6, 1, 5, 0,  7, 2, 4, 3  // z axis
    };

    // lookupEdgeIndexWithinEdgeNodes: for each axis, the index that the edge 4 nodes have in common has in each of them
    private static readonly int[] edgeIndexWithinEdgeNodes = {
        3,  2, 1,  0, // x axis
        7,  6, 5,  4, // y axis
        11, 9, 10, 8  // z axis
    };

    // lookupEdgeIndicesWithinFacePair: for each axis, the indices that the 4 edges around the face two nodes of the same
    // size have in common have in the first node and in the second
    private static readonly int[] edgeIndicesWithinFacePair = {
        5, 7,  9, 11,  4, 6, 8, 10, // x axis
        2, 3, 10, 11,  0, 1, 8,  9, // y axis
        1, 3,  6,  7,  0, 2, 4,  5  // z axis
    };

    public static int lookupEdgeNodeWithinNode( int axis, int group, int place ) {
        return edgeNodesWithinNode[( ( ( axis * 2 ) + group ) * 4 ) + place];
    }

    public static int lookupFacePairWithinNode( int axis, int pair, int side ) {
        return facePairsWithinNode[( ( ( axis * 4 ) + pair ) * 2 ) + side];
    }

    public static int lookupFacePairWithinFacePair( int axis, int pair, int side ) {
        return facePairsWithinFacePairs[( ( ( axis * 4 ) + pair ) * 2 ) + side];
    }

    public static int lookupEdgeNodeWithinFacePairChild( int axis, int edge, int place ) {
        return edgeNodesWithinFacePairsChildren[( ( ( axis * 4 ) + edge ) * 4 ) + place];
    }

    public static int lookupEdgeNodeWithinFacePairNode( int axis, int edge, int place ) {
        return edgeNodesWithinFacePairsNodes[( ( ( axis * 4 ) + edge ) * 4 ) + place];
    }

    public static int lookupEdgeNodeWithinFacePairAxis( int axis, int edge ) {
        return edgeNodesWithinFacePairsAxes[( axis * 4 ) + edge];
    }

    public static int lookupEdgeNodeWithinEdgeNodes( int axis, int group, int place ) {
        return edgeNodesWithinEdgeNodes[( ( ( axis * 2 ) + group ) * 4 ) + place];
    }

    public static int lookupEdgeIndexWithinEdgeNodes( int axis, int place ) {
        return edgeIndexWithinEdgeNodes[( axis * 4 ) + place];
    }

    public static int lookupEdgeIndexWithinFacePair( int axis, int side, int edge ) {
        return edgeIndicesWithinFacePair[( ( ( axis * 2 ) + side ) * 4 ) + edge];
    }

}
