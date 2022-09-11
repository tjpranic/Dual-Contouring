using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

public class Octree<T> {

    public T            data     { get; }
    public Octree<T>[]? children { get; private set; }

    protected Octree( T data ) {
        this.data     = data;
        this.children = new Octree<T>[8];
    }

    public static Octree<T> build( T data, Func<Octree<T>, int, T?> builder ) {
        var node = new Octree<T>( data );

        for( var childIndex = 0; childIndex < node.children!.Length; ++childIndex ) {
            var childData = builder( node, childIndex );
            if( childData != null ) {
                node.children[childIndex] = build( childData, builder );
            }
        }

        // TODO: improve
        if( node.children.All( ( child ) => child == null ) ) {
            node.children = null;
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

    public static IEnumerable<T> flatten( Octree<T> octree ) {
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