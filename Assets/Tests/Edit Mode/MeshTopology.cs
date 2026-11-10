using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// how the triangles of an indexed mesh share edges and vertices
public class MeshTopology {

    private class DisjointSets {

        private readonly Dictionary<int, int> parents = new( );

        public int count {
            get { return this.parents.Count( ( entry ) => entry.Key == entry.Value ); }
        }

        public int find( int item ) {
            if( !this.parents.ContainsKey( item ) ) {
                this.parents.Add( item, item );
            }
            while( this.parents[item] != item ) {
                item = this.parents[item];
            }
            return item;
        }

        public void union( int item, int other ) {
            this.parents[this.find( item )] = this.find( other );
        }

    }

    public int triangles           { get; private set; } // every triangle in the index buffer
    public int degenerateTriangles { get; private set; } // triangles that use a vertex more than once, left out of everything below
    public int vertices            { get; private set; } // vertices used by a triangle
    public int edges               { get; private set; } // edges used by a triangle
    public int openEdges           { get; private set; } // edges used by exactly one triangle
    public int nonManifoldEdges    { get; private set; } // edges used by more than two triangles
    public int oddEdges            { get; private set; } // edges used by an odd number of triangles
    public int windingConflicts    { get; private set; } // edges that more than one triangle runs along in the same direction
    public int pinchedVertices     { get; private set; } // vertices whose triangles form more than one fan
    public int components          { get; private set; } // groups of triangles connected through shared vertices

    public int eulerCharacteristic {
        get { return this.vertices - this.edges + ( this.triangles - this.degenerateTriangles ); }
    }

    public static MeshTopology of( Mesh mesh ) {
        var topology = new MeshTopology( );
        var indices  = mesh.triangles;

        var faces = new List<int[]>( );
        for( var index = 0; index < indices.Length; index += 3 ) {
            var face = new int[] { indices[index], indices[index + 1], indices[index + 2] };

            ++topology.triangles;
            if( face[0] == face[1] || face[1] == face[2] || face[0] == face[2] ) {
                ++topology.degenerateTriangles;
                continue;
            }
            faces.Add( face );
        }

        var edgeFaces     = new Dictionary<( int, int ), List<int>>( ); // faces on either side of each edge
        var directedEdges = new Dictionary<( int, int ), int>( );       // how many faces run along each edge in each direction
        var vertexFaces   = new Dictionary<int, List<int>>( );          // faces around each vertex

        for( var faceIndex = 0; faceIndex < faces.Count; ++faceIndex ) {
            var face = faces[faceIndex];

            for( var corner = 0; corner < 3; ++corner ) {
                var start = face[corner];
                var end   = face[( corner + 1 ) % 3];
                var edge  = start < end ? ( start, end ) : ( end, start );

                if( !edgeFaces.ContainsKey( edge ) ) {
                    edgeFaces.Add( edge, new( ) );
                }
                edgeFaces[edge].Add( faceIndex );

                directedEdges.TryGetValue( ( start, end ), out var count );
                directedEdges[( start, end )] = count + 1;

                if( !vertexFaces.ContainsKey( start ) ) {
                    vertexFaces.Add( start, new( ) );
                }
                vertexFaces[start].Add( faceIndex );
            }
        }

        topology.vertices         = vertexFaces.Count;
        topology.edges            = edgeFaces.Count;
        topology.openEdges        = edgeFaces.Values.Count( ( around ) => around.Count == 1 );
        topology.nonManifoldEdges = edgeFaces.Values.Count( ( around ) => around.Count > 2 );
        topology.oddEdges         = edgeFaces.Values.Count( ( around ) => around.Count % 2 == 1 );
        topology.windingConflicts = directedEdges.Values.Count( ( count ) => count > 1 );

        // the faces around a vertex form a single fan when each can be reached from any other by stepping across the
        // edges that meet at the vertex
        foreach( var entry in vertexFaces ) {
            var vertex = entry.Key;
            var fans   = new DisjointSets( );

            foreach( var faceIndex in entry.Value ) {
                foreach( var other in faces[faceIndex].Where( ( index ) => index != vertex ) ) {
                    var edge = vertex < other ? ( vertex, other ) : ( other, vertex );

                    foreach( var neighbor in edgeFaces[edge] ) {
                        fans.union( faceIndex, neighbor );
                    }
                }
            }

            if( fans.count > 1 ) {
                ++topology.pinchedVertices;
            }
        }

        var connected = new DisjointSets( );
        foreach( var face in faces ) {
            connected.union( face[0], face[1] );
            connected.union( face[1], face[2] );
        }
        topology.components = connected.count;

        return topology;
    }

    public override string ToString( ) {
        return
            $"{this.triangles} triangles ({this.degenerateTriangles} degenerate), {this.vertices} vertices, {this.edges} edges " +
            $"({this.openEdges} open, {this.nonManifoldEdges} non-manifold, {this.oddEdges} odd, {this.windingConflicts} winding conflicts), " +
            $"{this.pinchedVertices} pinched vertices, {this.components} components, euler characteristic {this.eulerCharacteristic}";
    }

}
