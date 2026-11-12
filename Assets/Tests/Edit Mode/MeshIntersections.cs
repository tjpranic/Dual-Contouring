using System;
using System.Collections.Generic;
using UnityEngine;

// where a mesh passes through itself: the pairs of its triangles of which a side of one goes through the inside of the
// other. Triangles that share a side are neighbors and are not looked at, and of two that share a corner only the sides
// opposite that corner are. Triangles that only touch, or that lie in the same plane, do not count
public static class MeshIntersections {

    private struct Point {

        public double x, y, z;

        public Point( Vector3 vector ) {
            this.x = vector.x;
            this.y = vector.y;
            this.z = vector.z;
        }

        public Point( double x, double y, double z ) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Point operator -( Point a, Point b ) {
            return new Point( a.x - b.x, a.y - b.y, a.z - b.z );
        }

        public static double dot( Point a, Point b ) {
            return ( a.x * b.x ) + ( a.y * b.y ) + ( a.z * b.z );
        }

        public static Point cross( Point a, Point b ) {
            return new Point( ( a.y * b.z ) - ( a.z * b.y ), ( a.z * b.x ) - ( a.x * b.z ), ( a.x * b.y ) - ( a.y * b.x ) );
        }

    }

    // whether the segment passes through the inside of the triangle, away from its own ends and from the sides of the
    // triangle (Moller and Trumbore)
    private static bool passesThrough( Point start, Point end, Point a, Point b, Point c ) {
        const double margin = 1e-7;

        var direction = end - start;
        var ab        = b - a;
        var ac        = c - a;

        var p           = Point.cross( direction, ac );
        var determinant = Point.dot( ab, p );

        // a segment in the plane of the triangle, or a triangle without area
        if( Math.Abs( determinant ) < 1e-30 ) {
            return false;
        }

        var offset = start - a;
        var u      = Point.dot( offset, p ) / determinant;

        if( u <= margin || u >= 1.0 - margin ) {
            return false;
        }

        var q = Point.cross( offset, ab );
        var v = Point.dot( direction, q ) / determinant;

        if( v <= margin || u + v >= 1.0 - margin ) {
            return false;
        }

        var along = Point.dot( ac, q ) / determinant;

        return along > margin && along < 1.0 - margin;
    }

    // the pairs of triangles that pass through each other, as the places of their first indices among the triangles of
    // all sub-meshes
    public static List<( int, int )> find( Mesh mesh ) {
        var vertices = mesh.vertices;
        var indices  = new List<int>( );

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            indices.AddRange( mesh.GetTriangles( subMesh ) );
        }

        return find( vertices, indices.ToArray( ) );
    }

    public static List<( int, int )> find( Vector3[] vertices, int[] indices ) {
        var count  = indices.Length / 3;
        var points = Array.ConvertAll( vertices, ( vertex ) => new Point( vertex ) );
        var pairs  = new List<( int, int )>( );

        if( count == 0 ) {
            return pairs;
        }

        // triangles are sorted into the cells of a grid that are about as large as the largest of them, and only those
        // that have a cell in common are compared
        var minimum = new Vector3[count];
        var maximum = new Vector3[count];
        var size    = 0.0f;

        for( var triangle = 0; triangle < count; ++triangle ) {
            var a = vertices[indices[triangle * 3]];
            var b = vertices[indices[( triangle * 3 ) + 1]];
            var c = vertices[indices[( triangle * 3 ) + 2]];

            minimum[triangle] = Vector3.Min( a, Vector3.Min( b, c ) );
            maximum[triangle] = Vector3.Max( a, Vector3.Max( b, c ) );

            var extent = maximum[triangle] - minimum[triangle];

            size = Mathf.Max( size, Mathf.Max( extent.x, Mathf.Max( extent.y, extent.z ) ) );
        }

        if( size <= 0.0f ) {
            return pairs;
        }

        var cells = new Dictionary<( int, int, int ), List<int>>( );

        for( var triangle = 0; triangle < count; ++triangle ) {
            for( var x = Mathf.FloorToInt( minimum[triangle].x / size ); x <= Mathf.FloorToInt( maximum[triangle].x / size ); ++x ) {
                for( var y = Mathf.FloorToInt( minimum[triangle].y / size ); y <= Mathf.FloorToInt( maximum[triangle].y / size ); ++y ) {
                    for( var z = Mathf.FloorToInt( minimum[triangle].z / size ); z <= Mathf.FloorToInt( maximum[triangle].z / size ); ++z ) {
                        if( !cells.TryGetValue( ( x, y, z ), out var list ) ) {
                            list = new List<int>( );

                            cells.Add( ( x, y, z ), list );
                        }

                        list.Add( triangle );
                    }
                }
            }
        }

        var seen = new HashSet<long>( );

        foreach( var list in cells.Values ) {
            for( var first = 0; first < list.Count; ++first ) {
                for( var second = first + 1; second < list.Count; ++second ) {
                    var one = list[first];
                    var two = list[second];

                    if( !seen.Add( ( ( long )one * count ) + two ) ) {
                        continue;
                    }

                    if( cross( points, indices, one, two ) ) {
                        pairs.Add( ( one, two ) );
                    }
                }
            }
        }

        return pairs;
    }

    public static int count( Mesh mesh ) {
        return find( mesh ).Count;
    }

    private static bool cross( Point[] points, int[] indices, int one, int two ) {
        // the corners the two triangles have in common, by where each is among the three of its triangle
        var shared      = 0;
        var sharedInOne = -1;
        var sharedInTwo = -1;

        for( var corner = 0; corner < 3; ++corner ) {
            for( var other = 0; other < 3; ++other ) {
                if( indices[( one * 3 ) + corner] == indices[( two * 3 ) + other] ) {
                    ++shared;

                    sharedInOne = corner;
                    sharedInTwo = other;
                }
            }
        }

        if( shared >= 2 ) {
            return false;
        }

        return sidesPassThrough( points, indices, one, shared == 1 ? sharedInOne : -1, two ) || sidesPassThrough( points, indices, two, shared == 1 ? sharedInTwo : -1, one );
    }

    // whether a side of the first triangle passes through the second: every side, or only the one opposite the given
    // corner
    private static bool sidesPassThrough( Point[] points, int[] indices, int triangle, int oppositeOnly, int other ) {
        var a = points[indices[other * 3]];
        var b = points[indices[( other * 3 ) + 1]];
        var c = points[indices[( other * 3 ) + 2]];

        for( var corner = 0; corner < 3; ++corner ) {
            if( oppositeOnly != -1 && corner != oppositeOnly ) {
                continue;
            }

            // the side opposite the corner
            var start = points[indices[( triangle * 3 ) + ( ( corner + 1 ) % 3 )]];
            var end   = points[indices[( triangle * 3 ) + ( ( corner + 2 ) % 3 )]];

            if( passesThrough( start, end, a, b, c ) ) {
                return true;
            }
        }

        return false;
    }

}
