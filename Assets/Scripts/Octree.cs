using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

public class Octree<T> {

    public T            data     { get; set; }
    public Octree<T>[]? children { get; set; }

    protected Octree( T data ) {
        this.data = data;
    }

    public static Octree<T> build( T data, Func<T, T[]?> builder ) {
        var node = new Octree<T>( data );

        node.children = builder( node.data )?.Select( ( data ) => new Octree<T>( data ) ).ToArray( );

        if( node.children != null ) {
            if( node.children.Length != 8 ) {
                throw new Exception( "Exactly 8 child nodes expected" );
            }

            for( var childIndex = 0; childIndex < node.children.Length; ++childIndex ) {
                node.children[childIndex] = build( node.children[childIndex].data, builder );
            }
        }

        return node;
    }

    public static void walk( Octree<T> octree, Action<Octree<T>> walker ) {
        walker( octree );

        if( octree.children != null ) {
            foreach( var child in octree.children ) {
                walk( child, walker );
            }
        }
    }

    public static Y reduce<Y>( Octree<T> octree, Y accumulator, Func<Y, Octree<T>, Y> reducer ) {
        accumulator = reducer( accumulator, octree );

        if( octree.children != null ) {
            foreach( var child in octree.children ) {
                accumulator = reduce( child, accumulator, reducer );
            }
        }

        return accumulator;
    }

    public static List<T> flatten( Octree<T> octree ) {
        return reduce(
            octree,
            new List<T>( ),
            ( accumulator, node ) => {
                accumulator.Add( node.data );
                return accumulator;
            }
        );
    }

}

#nullable disable