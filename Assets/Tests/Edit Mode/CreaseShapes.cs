using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;

// meshes built by hand for the tests of the creases, BurstCreasesTest and GPUCreasesTest, which run the same ones
// through both passes. Each comes with the normals its vertices start with
public static class CreaseShapes {

    // a mesh by hand: corners at the same position are one vertex, which keeps the normal it was first given
    public class Builder {

        public readonly List<float3>             positions = new( );
        public readonly List<float3>             normals   = new( );
        public readonly List<BurstMesh.Triangle> triangles = new( );

        private readonly Dictionary<float3, int> indices = new( );

        public int vertex( float3 position, float3 normal = default ) {
            if( !this.indices.TryGetValue( position, out var index ) ) {
                index = this.positions.Count;

                this.positions.Add( position );
                this.normals.Add( normal );
                this.indices.Add( position, index );
            }
            return index;
        }

        // a vertex that no triangle uses, even if another is at its position
        public int loose( float3 position, float3 normal ) {
            this.positions.Add( position );
            this.normals.Add( normal );

            return this.positions.Count - 1;
        }

        public void triangle( float3 a, float3 b, float3 c, int subMesh = 0 ) {
            this.triangles.Add( new BurstMesh.Triangle( this.vertex( a ), this.vertex( b ), this.vertex( c ), subMesh ) );
        }

        // the triangle turned to face away from a point inside the shape it is part of
        public void triangleFacingAwayFrom( float3 inside, float3 a, float3 b, float3 c, int subMesh = 0 ) {
            if( math.dot( math.cross( b - a, c - a ), ( ( a + b + c ) / 3.0f ) - inside ) < 0.0f ) {
                this.triangle( a, c, b, subMesh );
            }
            else {
                this.triangle( a, b, c, subMesh );
            }
        }

        // two triangles, a b c and a c d
        public void quad( float3 a, float3 b, float3 c, float3 d, int subMesh = 0 ) {
            this.triangle( a, b, c, subMesh );
            this.triangle( a, c, d, subMesh );
        }

        // the box from a corner by a size, one quad a face, facing out. Every vertex is given the normal along the
        // diagonal from the box's middle, which no face has. The sub-mesh of each face is given its axis and side
        public Builder box( float3 minimum, float3 size, Func<int, int, int> subMeshOf = null ) {
            var middle = minimum + ( size / 2.0f );

            for( var axis = 0; axis < 3; ++axis )
            for( var side = 0; side < 2; ++side ) {
                var u = ( axis + 1 ) % 3;
                var v = ( axis + 2 ) % 3;

                float3 corner( int i, int j ) {
                    var position = minimum;

                    position[axis] += side * size[axis];
                    position[u]    += i * size[u];
                    position[v]    += j * size[v];

                    this.vertex( position, math.normalize( position - middle ) );

                    return position;
                }

                var subMesh = subMeshOf?.Invoke( axis, side ) ?? 0;

                if( side == 1 ) {
                    this.quad( corner( 0, 0 ), corner( 1, 0 ), corner( 1, 1 ), corner( 0, 1 ), subMesh );
                }
                else {
                    this.quad( corner( 0, 0 ), corner( 0, 1 ), corner( 1, 1 ), corner( 1, 0 ), subMesh );
                }
            }
            return this;
        }

    }

    // a prism of a number of sides around the y axis, from y = 0 to y = 1, with fans for caps. Each side is cut into
    // two triangles along the same diagonal
    public static Builder prism( int sides ) {
        var builder = new Builder( );
        var bottom  = new float3( 0.0f, 0.0f, 0.0f );
        var top     = new float3( 0.0f, 1.0f, 0.0f );

        // the last side ends where the first begins: sin( 2 pi ) is not 0 in doubles
        float3 rim( int side, float height ) {
            var turn = 2.0 * math.PI_DBL * ( side % sides ) / sides;

            return new float3( ( float )math.cos( turn ), height, ( float )math.sin( turn ) );
        }

        for( var side = 0; side < sides; ++side ) {
            var b0 = rim( side, 0.0f );
            var b1 = rim( side + 1, 0.0f );
            var t0 = rim( side, 1.0f );
            var t1 = rim( side + 1, 1.0f );

            builder.triangle( top, t1, t0 );
            builder.triangle( bottom, b0, b1 );
            builder.quad( b0, t0, t1, b1 );
        }
        return builder;
    }

    // a patch of 8 by 8 quads of all shapes in the plane y = 0, whose vertices start with the patch's normal
    public static Builder flatPatch( ) {
        var builder = new Builder( );
        var random  = new Unity.Mathematics.Random( 7 );
        var up      = new float3( 0.0f, 1.0f, 0.0f );
        var points  = new float3[9, 9];

        for( var i = 0; i < 9; ++i )
        for( var j = 0; j < 9; ++j ) {
            points[i, j] = new float3( i + random.NextFloat( -0.3f, 0.3f ), 0.0f, j + random.NextFloat( -0.3f, 0.3f ) );

            builder.vertex( points[i, j], up );
        }

        for( var i = 0; i < 8; ++i )
        for( var j = 0; j < 8; ++j ) {
            builder.quad( points[i, j], points[i, j + 1], points[i + 1, j + 1], points[i + 1, j] );
        }
        return builder;
    }

    // a sphere of radius 1 of 48 sides and 24 rings, 7.5 degrees from one triangle to the next, whose vertices start with
    // the sphere's normals
    public static Builder sphere( ) {
        const int sides = 48;
        const int rings = 24;

        var builder = new Builder( );

        float3 at( int ring, int side ) {
            if( ring == 0 ) {
                return new float3( 0.0f, 1.0f, 0.0f );
            }
            if( ring == rings ) {
                return new float3( 0.0f, -1.0f, 0.0f );
            }

            var down   = math.PI_DBL * ring / rings;
            var around = 2.0 * math.PI_DBL * ( side % sides ) / sides;

            var position = new float3( ( float )( math.sin( down ) * math.cos( around ) ), ( float )math.cos( down ), ( float )( math.sin( down ) * math.sin( around ) ) );

            builder.vertex( position, math.normalize( position ) );

            return position;
        }

        builder.vertex( at( 0, 0 ), new float3( 0.0f, 1.0f, 0.0f ) );
        builder.vertex( at( rings, 0 ), new float3( 0.0f, -1.0f, 0.0f ) );

        for( var ring = 0; ring < rings; ++ring )
        for( var side = 0; side < sides; ++side ) {
            var a = at( ring, side );
            var b = at( ring, side + 1 );
            var c = at( ring + 1, side + 1 );
            var d = at( ring + 1, side );

            if( ring != 0 ) {
                builder.triangleFacingAwayFrom( float3.zero, a, b, c );
            }
            if( ring != rings - 1 ) {
                builder.triangleFacingAwayFrom( float3.zero, a, c, d );
            }
        }
        return builder;
    }

    // the tip of a cone of 24 sides whose faces lean 60 degrees from its axis, 13 degrees from one face to the next
    public static Builder coneTip( out float3 tip ) {
        const int sides = 24;

        var builder = new Builder( );
        var apex    = new float3( 0.0f, ( float )( 1.0 / math.tan( math.radians( 30.0 ) ) ), 0.0f );
        var inside  = new float3( 0.0f, apex.y / 3.0f, 0.0f );

        float3 rim( int side ) {
            var turn = 2.0 * math.PI_DBL * ( side % sides ) / sides;

            return new float3( ( float )math.cos( turn ), 0.0f, ( float )math.sin( turn ) );
        }

        for( var side = 0; side < sides; ++side ) {
            builder.triangleFacingAwayFrom( inside, apex, rim( side ), rim( side + 1 ) );
        }

        tip = apex;

        return builder;
    }

    // four triangles at the origin whose normals turn 0, 39, 58.5 and 71 degrees about the y axis, with corners there of
    // 1, 0.99, 0.98 and 0.97 radians, each larger than the one before. Taken largest corner first each is within 40
    // degrees of the group's mean when it joins, but the normal of the group of all four, which weighs them by their
    // areas too, is 60.8 degrees from the first (finding 11)
    public static Builder turningTriangles( ) {
        return fanAboutY( new[] { 0.0, 39.0, 58.5, 71.0 }, new[] { 1.0, 0.99, 0.98, 0.97 }, new[] { 1.0, 2.0, 3.0, 4.0 } );
    }

    // triangles at the origin whose normals turn about the y axis by angles in degrees from (0, 0, 1), with corners
    // there in radians, and two sides there as long as their scales
    private static Builder fanAboutY( double[] turns, double[] corners, double[] scales ) {
        var builder = new Builder( );
        var axis    = new double3( 0.0, 1.0, 0.0 );

        for( var index = 0; index < turns.Length; ++index ) {
            var normal = new double3( math.sin( math.radians( turns[index] ) ), 0.0, math.cos( math.radians( turns[index] ) ) );
            var across = math.cross( normal, axis );
            var scale  = scales[index];

            var b = axis * scale;
            var c = ( ( axis * math.cos( corners[index] ) ) + ( across * math.sin( corners[index] ) ) ) * scale;

            builder.triangle( float3.zero, ( float3 )b, ( float3 )c );
        }
        return builder;
    }

    // three triangles at the origin whose normals turn -20, 10 and 30 degrees about the y axis, with corners there of
    // 1, 0.9 and 0.5 radians and sides there 1, 1.2 and 10 long. Weighed by their angles alone the three make one group
    // whose mean is within 29 degrees of each, but the group's normal, weighed by their areas too, is 48 degrees from
    // the first
    public static Builder smallTrianglesBesideALargeOne( ) {
        return fanAboutY( new[] { -20.0, 10.0, 30.0 }, new[] { 1.0, 0.9, 0.5 }, new[] { 1.0, 1.2, 10.0 } );
    }

    // three triangles at the origin whose normals turn 0, 35 and 72 degrees about the y axis, with corners there of 1,
    // 0.9 and 0.5 radians and sides there 1, 10 and 1.2 long. The first two make a group whose mean, weighed by their
    // angles, is 55 degrees from the third. Weighed by their areas too it would be 37 degrees from it
    public static Builder largeTriangleBetweenSmallOnes( ) {
        return fanAboutY( new[] { 0.0, 35.0, 72.0 }, new[] { 1.0, 0.9, 0.5 }, new[] { 1.0, 10.0, 1.2 } );
    }

    // three triangles at the origin with corners there of exactly 90 degrees and normals 45 degrees apart, (0, -1, 0),
    // (1, -1, 0) / sqrt( 2 ) and (1, 0, 0), in that order or the other way round
    public static Builder cornersOfTheSameSize( bool reversed ) {
        var builder   = new Builder( );
        var triangles = new[] {
            ( new float3( 1.0f, 0.0f, 0.0f ), new float3( 0.0f, 0.0f, 1.0f ) ),
            ( new float3( 1.0f, 1.0f, 0.0f ), new float3( 0.0f, 0.0f, 2.0f ) ),
            ( new float3( 0.0f, 1.0f, 0.0f ), new float3( 0.0f, 0.0f, 3.0f ) )
        };

        foreach( var ( b, c ) in reversed ? triangles.Reverse( ) : triangles ) {
            builder.triangle( float3.zero, b, c );
        }
        return builder;
    }

    // two triangles at the origin with corners there of exactly 90 degrees, whose normals are (0, 0, 1) and 30 degrees
    // from it towards -y, the second with sides twice as long and four times the area. The vertices start without
    // normals, but for the origin when it is given a normal turned that way by an angle in degrees
    public static Builder trianglesOfTwoSizes( double? origin = null ) {
        var builder = new Builder( );
        var turn    = math.radians( 30.0 );

        builder.triangle( float3.zero, new float3( 1.0f, 0.0f, 0.0f ), new float3( 0.0f, 1.0f, 0.0f ) );
        builder.triangle( float3.zero, new float3( 0.0f, ( float )math.cos( turn ), ( float )math.sin( turn ) ) * 2.0f, new float3( -2.0f, 0.0f, 0.0f ) );

        if( origin.HasValue ) {
            builder.normals[0] = new float3( 0.0f, -( float )math.sin( math.radians( origin.Value ) ), ( float )math.cos( math.radians( origin.Value ) ) );
        }
        return builder;
    }

    // a normal tilted from y towards x by an angle in degrees
    public static float3 tilted( double degrees ) {
        return new float3( ( float )math.sin( math.radians( degrees ) ), ( float )math.cos( math.radians( degrees ) ), 0.0f );
    }

    // a flat patch of 3 by 3 squares whose vertices start with its normal, but for four in the middle, one without a
    // normal, one flipped, one 60 degrees off and one 10, and one on its edge a twentieth of a degree off. Gives those
    // five with the normals they start with
    public static Builder patchWithOddNormals( out Dictionary<int, float3> odd ) {
        var builder = new Builder( );
        var up      = new float3( 0.0f, 1.0f, 0.0f );

        for( var i = 0; i < 4; ++i )
        for( var j = 0; j < 4; ++j ) {
            builder.vertex( new float3( i, 0.0f, j ), up );
        }

        for( var i = 0; i < 3; ++i )
        for( var j = 0; j < 3; ++j ) {
            builder.quad( new float3( i, 0.0f, j ), new float3( i, 0.0f, j + 1 ), new float3( i + 1, 0.0f, j + 1 ), new float3( i + 1, 0.0f, j ) );
        }

        odd = new Dictionary<int, float3> {
            [builder.vertex( new float3( 1.0f, 0.0f, 1.0f ) )] = float3.zero,
            [builder.vertex( new float3( 1.0f, 0.0f, 2.0f ) )] = -up,
            [builder.vertex( new float3( 2.0f, 0.0f, 1.0f ) )] = tilted( 60.0 ),
            [builder.vertex( new float3( 2.0f, 0.0f, 2.0f ) )] = tilted( 10.0 ),
            [builder.vertex( new float3( 3.0f, 0.0f, 2.0f ) )] = tilted( 0.05 )
        };

        foreach( var ( vertex, normal ) in odd ) {
            builder.normals[vertex] = normal;
        }
        return builder;
    }

    // a cube of side 1 with a triangle too flat to have a normal along the edge from (0, 0, 0) to (1, 0, 0), through a
    // vertex of its own halfway, a millionth off the edge, whose only triangle it is. The largest sine of its angles is
    // 6e-6, under BurstCreases.flatness, and the normal its corners would give lies 45 degrees from both faces at the edge
    public static Builder cubeWithFlatTriangle( ) {
        var builder = new Builder( ).box( float3.zero, new float3( 1.0f ) );
        var halfway = new float3( 0.5f, 1e-6f, 1e-6f );

        builder.vertex( halfway, math.normalize( new float3( 0.0f, -1.0f, -1.0f ) ) );
        builder.triangle( new float3( 0.0f, 0.0f, 0.0f ), halfway, new float3( 1.0f, 0.0f, 0.0f ) );

        return builder;
    }

    // two cubes of side 1, with a vertex no triangle uses between them, at (5, 6, 7), vertex 8, with a normal given
    public static Builder cubesWithALooseVertex( float3 normal ) {
        var builder = new Builder( );

        builder.box( float3.zero, new float3( 1.0f ) );
        builder.loose( new float3( 5.0f, 6.0f, 7.0f ), normal );
        builder.box( new float3( 2.0f, 0.0f, 0.0f ), new float3( 1.0f ) );

        return builder;
    }

    // two boxes that meet along an edge of four triangles, and two more that meet at a corner, (3, 0, 0)
    public static Builder boxesAlongAnEdgeAndAtACorner( ) {
        var builder = new Builder( );

        builder.box( new float3( 0.0f, 0.0f, 0.0f ), new float3( 1.0f ) );
        builder.box( new float3( -1.0f, -1.0f, 0.0f ), new float3( 1.0f ) );
        builder.box( new float3( 3.0f, 0.0f, 0.0f ), new float3( 1.0f ) );
        builder.box( new float3( 2.0f, -1.0f, -1.0f ), new float3( 1.0f ) );

        return builder;
    }

}
