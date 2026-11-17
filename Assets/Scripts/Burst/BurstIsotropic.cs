using System;
using Unity.Collections;
using Unity.Mathematics;

using Connectivity = BurstMeshSimplifier.Connectivity;
using FlipRefusal  = BurstMeshSimplifier.FlipRefusal;
using Corner       = BurstFeatures.Corner;

// the isotropic remesh of Botsch and Kobbelt (A Remeshing Approach to Multiresolution Modeling, section 4): edges that
// are too long are split, edges that are too short collapsed, edges flipped towards vertices of six edges, and vertices
// smoothed. The paper remeshes a smooth surface and leaves out features, which collapses and flips are allowed and
// staying on the surface. What this has of those is the plan's own (Documents/Plans/Remeshing.md, finding 9).
//
// So far the mesh it works on and where a collapse goes. The loop comes with step 3 of the plan
public static class BurstIsotropic {

    // what a vertex is to a remesh, from the features it has now (BurstFeatures.isCorner). A corner stays where it
    // is, a vertex of a line moves along the line, and a vertex of the surface within its tangent plane
    public enum Kind : byte {
        Surface,
        Line,
        Corner
    }

    // the mesh a remesh works on: the simplifier's lists, and beside them the features of BurstFeatures, which the
    // simplifier's own sharp edges are among and which split, flip and collapse carry along. The lists of the features
    // are as long as the connectivity's, edge for edge and vertex for vertex
    public struct FeatureMesh : IDisposable {

        public Connectivity mesh;

        // for every edge, the line of the mesh as it came that the edge is on, or -1
        public NativeList<int> edgeLines;

        // for every vertex, the corners found by angle on the mesh as it came
        public NativeList<Corner> corners;

        public FeatureMesh( int vertexCount, int triangleCount, Allocator allocator ) {
            this.mesh      = new Connectivity( vertexCount, triangleCount, allocator );
            this.edgeLines = new NativeList<int>( triangleCount * 3 / 2, allocator );
            this.corners   = new NativeList<Corner>( vertexCount, allocator );
        }

        public void Dispose( ) {
            this.mesh.Dispose( );
            this.edgeLines.Dispose( );
            this.corners.Dispose( );
        }

        public readonly int featureEdgesOf( int vertex ) {
            return BurstFeatures.featureEdgesOf( this.mesh, this.edgeLines, vertex );
        }

        public readonly Kind kindOf( int vertex ) {
            var count = this.featureEdgesOf( vertex );

            if( BurstFeatures.isCorner( this.mesh.isPoint( vertex ), count, this.corners[vertex] ) ) {
                return Kind.Corner;
            }

            return count == 2 ? Kind.Line : Kind.Surface;
        }

        // splits an edge at a position (Connectivity.split). The two halves of a feature edge are on its line, and the
        // edges across its triangles are on none. Gives the new vertex
        public int split( int edge, double3 position ) {
            var line   = this.edgeLines[edge];
            var middle = this.mesh.split( edge, position );

            // the first edge added is the second half, and the edges across the triangles follow
            this.edgeLines.Add( line );

            while( this.edgeLines.Length < this.mesh.edgeCount ) {
                this.edgeLines.Add( -1 );
            }

            this.corners.Add( Corner.None );

            return middle;
        }

        // whether an edge may be flipped (Connectivity.checkFlip), at a feature angle given by its cosine: never a
        // feature edge, and never where the two triangles the flip makes would meet at more than the angle, which
        // would make a feature where there was none (finding 9)
        public readonly FlipRefusal checkFlip( int edge, double cosine ) {
            if( this.edgeLines[edge] != -1 && this.mesh.edgeTriangles[edge] == 2 && this.mesh.edgeSharp[edge] == BurstMeshSimplifier.Sharp.None ) {
                return FlipRefusal.Feature;
            }

            return this.mesh.checkFlip( edge, cosine );
        }

        // flips an edge that checkFlip allows. It is no feature edge, and neither is the edge it becomes
        public void flip( int edge ) {
            this.mesh.flip( edge );
        }

        // where a collapse of an edge goes, by what its two ends are (finding 9):
        // - two vertices of the surface collapse into the middle of the edge, as the paper has it;
        // - a vertex of the surface collapses into the other end, which is on a line or a corner;
        // - along a line, two of its vertices collapse into the middle, and a vertex of the line into a corner;
        // - two corners along a line become one, where at most one of them is a point that the simplifier keeps: into
        //   that one, or else into the one where more lines meet, and of two alike into the lower.
        // An edge between two vertices of lines or corners that is not itself a feature edge does not collapse, nor
        // does one between two points. Gives whether the edge may collapse, the position, and the end it goes into, or
        // -1 for the middle
        public readonly bool placeCollapse( int edge, out double3 position, out int into ) {
            var a = this.mesh.edges[edge].x;
            var b = this.mesh.edges[edge].y;

            var kindA = this.kindOf( a );
            var kindB = this.kindOf( b );

            into     = -1;
            position = ( this.mesh.positions[a] + this.mesh.positions[b] ) * 0.5;

            if( kindA == Kind.Surface && kindB == Kind.Surface ) {
                return true;
            }

            if( kindA == Kind.Surface ) {
                into = b;
            }
            else if( kindB == Kind.Surface ) {
                into = a;
            }
            else if( this.edgeLines[edge] == -1 ) {
                return false;
            }
            else if( kindA == Kind.Line && kindB == Kind.Line ) {
                return true;
            }
            else if( kindA == Kind.Line ) {
                into = b;
            }
            else if( kindB == Kind.Line ) {
                into = a;
            }
            else {
                var pointA = this.mesh.isPoint( a );
                var pointB = this.mesh.isPoint( b );

                if( pointA && pointB ) {
                    return false;
                }

                if( pointA != pointB ) {
                    into = pointA ? a : b;
                }
                else {
                    var countA = this.featureEdgesOf( a );
                    var countB = this.featureEdgesOf( b );

                    into = countA != countB ? ( countA > countB ? a : b ) : math.min( a, b );
                }
            }

            position = this.mesh.positions[into];

            return true;
        }

        // the longest edge a collapse of an edge into a position would leave, from there to the vertices next to either
        // end. A collapse that leaves one longer than a split allows is not made, as the next split would only undo it
        // (finding 9)
        public readonly double longestEdgeAfter( int edge, double3 position ) {
            var a       = this.mesh.edges[edge].x;
            var b       = this.mesh.edges[edge].y;
            var longest = 0.0;

            for( var end = this.mesh.firstEdges[a]; end != -1; end = this.mesh.nextEdges[end] ) {
                var neighbour = this.mesh.across( end >> 1, a );

                if( neighbour != b ) {
                    longest = math.max( longest, math.distance( position, this.mesh.positions[neighbour] ) );
                }
            }
            for( var end = this.mesh.firstEdges[b]; end != -1; end = this.mesh.nextEdges[end] ) {
                var neighbour = this.mesh.across( end >> 1, b );

                if( neighbour != a ) {
                    longest = math.max( longest, math.distance( position, this.mesh.positions[neighbour] ) );
                }
            }

            return longest;
        }

        // collapses an edge that the connectivity's check allows into the position placeCollapse gave, and into the end
        // it gave (Connectivity.collapse). The kept vertex takes the corners of the end the collapse went into, or of
        // both for the middle. Of two edges that become one, at a vertex opposite the edge, the one that is left is on
        // the line either was on. Gives the vertex kept
        public int collapse( int edge, double3 position, int into ) {
            var a       = this.mesh.edges[edge].x;
            var b       = this.mesh.edges[edge].y;
            var kept    = math.min( a, b );
            var removed = math.max( a, b );

            this.corners[kept] = into == -1 ? this.corners[a] | this.corners[b] : this.corners[into];

            for( var corner = this.mesh.firstCorners[a]; corner != -1; corner = this.mesh.nextCorners[corner] ) {
                var triangle = this.mesh.triangles[corner / 3];

                if( !math.any( triangle == b ) ) {
                    continue;
                }

                var opposite  = triangle.x != a && triangle.x != b ? triangle.x : triangle.y != a && triangle.y != b ? triangle.y : triangle.z;
                var staying   = this.mesh.edgeBetween( kept, opposite );
                var going     = this.mesh.edgeBetween( removed, opposite );

                if( this.edgeLines[staying] == -1 ) {
                    this.edgeLines[staying] = this.edgeLines[going];
                }
            }

            var result = this.mesh.collapse( edge, position );

            for( var index = 0; index < this.mesh.removedEdges.Length; ++index ) {
                this.edgeLines[this.mesh.removedEdges[index]] = -1;
            }

            return result;
        }

    }

}
