using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

// a mesh built by hand for the remeshing tests, from triangles given by the positions of their corners. Corners at the
// same position are one vertex, so a shape built from rings of points has to make each point the same way each time
internal class HandBuiltMesh {

    public readonly List<Vector3>            positions = new( );
    public readonly List<BurstMesh.Triangle> triangles = new( );

    private readonly Dictionary<Vector3, int> indices = new( );

    public int vertex( Vector3 position ) {
        if( !this.indices.TryGetValue( position, out var index ) ) {
            index = this.positions.Count;

            this.positions.Add( position );
            this.indices.Add( position, index );
        }
        return index;
    }

    public int at( float x, float y, float z ) {
        Assert.IsTrue( this.indices.TryGetValue( new Vector3( x, y, z ), out var index ), $"a vertex at {x}, {y}, {z}" );

        return index;
    }

    public HandBuiltMesh triangle( Vector3 a, Vector3 b, Vector3 c, int subMesh = 0 ) {
        this.triangles.Add( new BurstMesh.Triangle( this.vertex( a ), this.vertex( b ), this.vertex( c ), subMesh ) );
        return this;
    }

    // two triangles, a b c and a c d, whose diagonal runs from a to c
    public HandBuiltMesh quad( Vector3 a, Vector3 b, Vector3 c, Vector3 d, int subMesh = 0 ) {
        this.triangle( a, b, c, subMesh );
        return this.triangle( a, c, d, subMesh );
    }

    // a flat patch of unit squares in the plane z = 0, facing up, with open edges all round. The sub-mesh of each
    // square is given its column and row
    public HandBuiltMesh patch( int width, int height, Func<int, int, int> subMeshOf = null ) {
        for( var i = 0; i < width; ++i )
        for( var j = 0; j < height; ++j ) {
            this.quad( new Vector3( i, j, 0 ), new Vector3( i + 1, j, 0 ), new Vector3( i + 1, j + 1, 0 ), new Vector3( i, j + 1, 0 ), subMeshOf == null ? 0 : subMeshOf( i, j ) );
        }
        return this;
    }

    // the surface of a box of unit cells, facing out. The sub-mesh of each square is given its centre and the axis and
    // direction it faces. Every square's diagonal runs from its lowest corner to its highest
    public HandBuiltMesh box( Vector3Int minimum, Vector3Int size, Func<Vector3, int, int, int> subMeshOf = null ) {
        for( var axis = 0; axis < 3; ++axis )
        for( var side = 0; side < 2; ++side ) {
            var u = ( axis + 1 ) % 3;
            var v = ( axis + 2 ) % 3;

            for( var i = 0; i < size[u]; ++i )
            for( var j = 0; j < size[v]; ++j ) {
                Vector3 corner( int di, int dj ) {
                    var position = Vector3.zero;

                    position[axis] = minimum[axis] + ( side * size[axis] );
                    position[u]    = minimum[u] + i + di;
                    position[v]    = minimum[v] + j + dj;

                    return position;
                }

                var subMesh = subMeshOf == null ? 0 : subMeshOf( ( corner( 0, 0 ) + corner( 1, 1 ) ) / 2, axis, ( side * 2 ) - 1 );

                if( side == 1 ) {
                    this.quad( corner( 0, 0 ), corner( 1, 0 ), corner( 1, 1 ), corner( 0, 1 ), subMesh );
                }
                else {
                    this.quad( corner( 0, 0 ), corner( 0, 1 ), corner( 1, 1 ), corner( 1, 0 ), subMesh );
                }
            }
        }
        return this;
    }

    // point k of a ring of n round the z axis at a height
    public static Vector3 onRing( int k, int n, float radius, float height ) {
        var angle = 2.0 * Math.PI * ( k % n ) / n;

        return new Vector3( ( float )( radius * Math.Cos( angle ) ), ( float )( radius * Math.Sin( angle ) ), height );
    }

    // a closed prism of n sides round the z axis, from z = 0 to the height, its two ends fans round their centres
    public HandBuiltMesh prism( int n, float radius, float height ) {
        for( var k = 0; k < n; ++k ) {
            this.quad( onRing( k, n, radius, 0.0f ), onRing( k + 1, n, radius, 0.0f ), onRing( k + 1, n, radius, height ), onRing( k, n, radius, height ) );
            this.triangle( new Vector3( 0.0f, 0.0f, height ), onRing( k, n, radius, height ), onRing( k + 1, n, radius, height ) );
            this.triangle( Vector3.zero, onRing( k + 1, n, radius, 0.0f ), onRing( k, n, radius, 0.0f ) );
        }
        return this;
    }

    // a closed cone of n sides round the z axis, its tip at the height, its base a fan round its centre
    public HandBuiltMesh cone( int n, float radius, float height ) {
        for( var k = 0; k < n; ++k ) {
            this.triangle( onRing( k, n, radius, 0.0f ), onRing( k + 1, n, radius, 0.0f ), new Vector3( 0.0f, 0.0f, height ) );
            this.triangle( Vector3.zero, onRing( k + 1, n, radius, 0.0f ), onRing( k, n, radius, 0.0f ) );
        }
        return this;
    }

    // a flat disc of n sides of sub-mesh 1 in the plane z = 0, facing up, in a ring of sub-mesh 0 out to an open rim
    public HandBuiltMesh disc( int n, float radius, float outerRadius ) {
        for( var k = 0; k < n; ++k ) {
            this.triangle( Vector3.zero, onRing( k, n, radius, 0.0f ), onRing( k + 1, n, radius, 0.0f ), 1 );
            this.quad( onRing( k, n, radius, 0.0f ), onRing( k, n, outerRadius, 0.0f ), onRing( k + 1, n, outerRadius, 0.0f ), onRing( k + 1, n, radius, 0.0f ) );
        }
        return this;
    }

    // the sum of the normals of the triangles of each vertex, weighted by their areas, made unit length
    public Vector3[] normals( ) {
        var normals = new Vector3[this.positions.Count];

        foreach( var triangle in this.triangles ) {
            var normal = Vector3.Cross( this.positions[triangle.b] - this.positions[triangle.a], this.positions[triangle.c] - this.positions[triangle.a] );

            normals[triangle.a] += normal;
            normals[triangle.b] += normal;
            normals[triangle.c] += normal;
        }

        return normals.Select( ( normal ) => normal.normalized ).ToArray( );
    }

    public NativeArray<float3> nativePositions( Allocator allocator = Allocator.TempJob ) {
        return new NativeArray<float3>( this.positions.Select( ( position ) => ( float3 )position ).ToArray( ), allocator );
    }

    public NativeArray<float3> nativeNormals( Allocator allocator = Allocator.TempJob ) {
        return new NativeArray<float3>( this.normals( ).Select( ( normal ) => ( float3 )normal ).ToArray( ), allocator );
    }

    public NativeArray<BurstMesh.Triangle> nativeTriangles( Allocator allocator = Allocator.TempJob ) {
        return new NativeArray<BurstMesh.Triangle>( this.triangles.ToArray( ), allocator );
    }

}
