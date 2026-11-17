using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

using Connectivity = BurstMeshSimplifier.Connectivity;
using Sharp        = BurstMeshSimplifier.Sharp;

// the features of a mesh that a remesh keeps (Documents/Plans/Remeshing.md, findings 6, 9 and 17): the edges where the
// surface sets itself apart, the corners where they end, meet or turn, and the lines they make from corner to corner.
// All three techniques find them the same way, on the mesh the pass is given.
//
// Features come in two strengths (finding 9). The simplifier's sharp edges, which are open, of more than two triangles
// or between materials, are hard: its lists hold their topology. The others are found by angle and hold only at the
// scale of the remesh, since an edge that crosses the grid makes noise of them (finding 17)
public static class BurstFeatures {

    // the corners found by angle on the mesh as it came, which a vertex carries through a remesh (finding 6)
    [Flags]
    public enum Corner : byte {
        None = 0,
        Turn = 1, // a line of two feature edges turns here by more than the angle
        Tip  = 2  // no feature edge, and the normals of the triangles are further than the angle from their mean
    }

    // whether a vertex is a corner: where the simplifier keeps a point of its own (Connectivity.isPoint) whatever the
    // angle; where the feature edges do not make one line through it, which is one of them, or three and more; where a
    // line turned when it was found; and where a tip was found and no line has come to it since. Going by the feature
    // edges a vertex has now, a corner of lines a remesh has collapsed away is no corner any more
    public static bool isCorner( bool point, int featureEdges, Corner found ) {
        if( point || ( featureEdges != 0 && featureEdges != 2 ) ) {
            return true;
        }

        return featureEdges == 2 ? ( found & Corner.Turn ) != 0 : ( found & Corner.Tip ) != 0;
    }

    // the normal of a triangle of the lists, times twice its area
    public static double3 normalOf( in Connectivity mesh, int triangle ) {
        var corners = mesh.triangles[triangle];

        return math.cross( mesh.positions[corners.y] - mesh.positions[corners.x], mesh.positions[corners.z] - mesh.positions[corners.x] );
    }

    // whether two triangles meet at more than the angle whose cosine is given. A triangle without area has no normal,
    // and meets nothing at an angle: both sides are then zero
    public static bool bent( double3 normal, double3 other, double cosine ) {
        return math.dot( normal, other ) < cosine * math.length( normal ) * math.length( other );
    }

    // how many feature edges a vertex has: edges whose line is not -1
    public static int featureEdgesOf( in Connectivity mesh, NativeList<int> edgeLines, int vertex ) {
        var count = 0;

        for( var end = mesh.firstEdges[vertex]; end != -1; end = mesh.nextEdges[end] ) {
            if( edgeLines[end >> 1] != -1 ) {
                ++count;
            }
        }
        return count;
    }

    // the features of a mesh as lists, at an angle given by its cosine:
    // - for every edge, the line it is on, or -1 where it is no feature edge. A line runs from corner to corner, or is
    //   a loop without one, and the lines are numbered in the order they are found: from the corners in the order of
    //   their vertices, and then the loops;
    // - for every vertex, the corners found by angle;
    // - how many lines there are.
    // The lists are made as long as the connectivity's
    [BurstCompile( CompileSynchronously = true, FloatMode = BurstFloats.mode, FloatPrecision = BurstFloats.precision )]
    public struct SkeletonJob : IJob {

        public double       cosine;
        public Connectivity mesh;

        public NativeList<int>    edgeLines;
        public NativeList<Corner> corners;

        // one
        [WriteOnly] public NativeArray<int> lineCount;

        // a feature edge before it has a line
        private const int Unnumbered = -2;

        public void Execute( ) {
            this.edgeLines.Clear( );
            this.corners.Clear( );

            this.edgeLines.Resize( this.mesh.edgeCount, NativeArrayOptions.UninitializedMemory );
            this.corners.Resize( this.mesh.vertexCount, NativeArrayOptions.ClearMemory );

            for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                this.edgeLines[edge] = this.isFeature( edge ) ? Unnumbered : -1;
            }

            // corners by angle, at the vertices of one line and at the vertices of none
            for( var vertex = 0; vertex < this.mesh.vertexCount; ++vertex ) {
                if( this.mesh.firstCorners[vertex] == -1 ) {
                    continue;
                }

                var count = featureEdgesOf( this.mesh, this.edgeLines, vertex );

                if( count == 2 && this.turns( vertex ) ) {
                    this.corners[vertex] = Corner.Turn;
                }
                else if( count == 0 && this.spreads( vertex ) ) {
                    this.corners[vertex] = Corner.Tip;
                }
            }

            // the lines from the corners, then the loops. Away from a corner a vertex has exactly two feature edges,
            // so a line goes on through it by the other one
            var lines = 0;

            for( var vertex = 0; vertex < this.mesh.vertexCount; ++vertex ) {
                if( this.mesh.firstCorners[vertex] == -1 || !this.cornerAt( vertex ) ) {
                    continue;
                }

                for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                    if( this.edgeLines[end >> 1] == Unnumbered ) {
                        this.walk( end >> 1, vertex, lines++ );
                    }
                }
            }

            for( var edge = 0; edge < this.mesh.edgeCount; ++edge ) {
                if( this.edgeLines[edge] == Unnumbered ) {
                    this.walk( edge, this.mesh.edges[edge].x, lines++ );
                }
            }

            this.lineCount[0] = lines;
        }

        private bool isFeature( int edge ) {
            if( this.mesh.edges[edge].x == -1 ) {
                return false;
            }
            if( this.mesh.edgeSharp[edge] != Sharp.None ) {
                return true;
            }

            var a = this.mesh.edges[edge].x;
            var b = this.mesh.edges[edge].y;

            var first  = -1;
            var second = -1;

            for( var corner = this.mesh.firstCorners[a]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                if( math.any( this.mesh.triangles[corner / 3] == b ) ) {
                    if( first == -1 ) {
                        first = corner / 3;
                    }
                    else {
                        second = corner / 3;
                    }
                }
            }

            // not sharp, so it has two triangles of one sub-mesh
            return bent( normalOf( this.mesh, first ), normalOf( this.mesh, second ), this.cosine );
        }

        private readonly bool cornerAt( int vertex ) {
            return isCorner( this.mesh.isPoint( vertex ), featureEdgesOf( this.mesh, this.edgeLines, vertex ), this.corners[vertex] );
        }

        // whether the line through a vertex of two feature edges turns by more than the angle: the two edges, from the
        // vertex, are closer than 180 degrees less the angle
        private readonly bool turns( int vertex ) {
            var directions = new double3x2( );
            var found      = 0;

            for( var end = this.mesh.firstEdges[vertex]; end != -1; end = this.mesh.nextEdges[end] ) {
                if( this.edgeLines[end >> 1] != -1 ) {
                    directions[found++] = this.mesh.positions[this.mesh.across( end >> 1, vertex )] - this.mesh.positions[vertex];
                }
            }

            var lengths = math.length( directions.c0 ) * math.length( directions.c1 );

            return lengths > 0.0 && math.dot( directions.c0, directions.c1 ) > -this.cosine * lengths;
        }

        // whether the normal of a triangle of a vertex is further than the angle from the mean of them all, weighted by
        // area, as at the tip of a cone
        private readonly bool spreads( int vertex ) {
            var mean = double3.zero;

            for( var corner = this.mesh.firstCorners[vertex]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                mean += normalOf( this.mesh, corner / 3 );
            }

            for( var corner = this.mesh.firstCorners[vertex]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                if( bent( normalOf( this.mesh, corner / 3 ), mean, this.cosine ) ) {
                    return true;
                }
            }
            return false;
        }

        // numbers the edges of a line, from an edge and the vertex it starts at, until a corner, or until it comes back
        // round to where it started
        private void walk( int edge, int vertex, int line ) {
            while( true ) {
                this.edgeLines[edge] = line;

                var next = this.mesh.across( edge, vertex );

                if( this.cornerAt( next ) ) {
                    return;
                }

                var following = -1;

                for( var end = this.mesh.firstEdges[next]; end != -1; end = this.mesh.nextEdges[end] ) {
                    if( this.edgeLines[end >> 1] == Unnumbered ) {
                        following = end >> 1;

                        break;
                    }
                }

                if( following == -1 ) {
                    return;
                }

                edge   = following;
                vertex = next;
            }
        }

    }

}
