using Unity.Mathematics;

// the octree of the Burst implementations. The managed implementations always build it full, so it is a grid of nodes
// for each level: level L has 2^L nodes across, the root is level 0 and the leaves are level D, a uniform grid of
// 2^D cells. Node (x, y, z) of a level has the children (2x + i, 2y + j, 2z + k) on the next one, in the child order
// of the managed octree, which is the order of the corner layout. The nodes of all levels are in one array, level
// by level, each level x outermost
public readonly struct BurstOctree {

    public readonly int depth;

    public BurstOctree( int depth ) {
        this.depth = depth;
    }

    // the uniform grid of the leaves, whose corners are the corners of every node
    public BurstGrid grid {
        get { return new BurstGrid( 1 << this.depth ); }
    }

    public int levelStart( int level ) {
        return ( ( 1 << ( 3 * level ) ) - 1 ) / 7;
    }

    public int nodeCount {
        get { return this.levelStart( this.depth + 1 ); }
    }

    // the level of the node at the given place in the array of all nodes
    public int levelOf( int index ) {
        var level = 0;

        while( this.levelStart( level + 1 ) <= index ) {
            ++level;
        }

        return level;
    }

    public int nodesAcross( int level ) {
        return 1 << level;
    }

    public int nodeIndex( int level, int3 node ) {
        var across = this.nodesAcross( level );
        return this.levelStart( level ) + ( ( ( ( node.x * across ) + node.y ) * across ) + node.z );
    }

    public int3 nodeOf( int level, int local ) {
        var across = this.nodesAcross( level );
        return new int3( local / ( across * across ), ( local / across ) % across, local % across );
    }

    // how many leaves across a node of the level is
    public int span( int level ) {
        return 1 << ( this.depth - level );
    }

    public int3 childOf( int3 node, int child ) {
        return ( node * 2 ) + new int3( BurstTables.lookupCornerOffset( child, 0 ), BurstTables.lookupCornerOffset( child, 1 ), BurstTables.lookupCornerOffset( child, 2 ) );
    }

    // the corner of the grid of the leaves that the given corner of a node is
    public int3 cornerOfNode( int level, int3 node, int corner ) {
        var offset = new int3( BurstTables.lookupCornerOffset( corner, 0 ), BurstTables.lookupCornerOffset( corner, 1 ), BurstTables.lookupCornerOffset( corner, 2 ) );
        return ( node + offset ) * this.span( level );
    }

    // the managed octree puts the center of a child a quarter of its parent away from the parent's center, which for
    // a power of two is exactly this
    public float3 centerOf( int level, int3 node ) {
        return ( ( new float3( node ) + 0.5f ) / this.nodesAcross( level ) ) - 0.5f;
    }

    public float sizeOf( int level ) {
        return 1.0f / this.nodesAcross( level );
    }

}

// a node of the octree: its level and where it is on that level
public struct BurstNode {

    public int  level;
    public int3 coordinates;

    public BurstNode( int level, int3 coordinates ) {
        this.level       = level;
        this.coordinates = coordinates;
    }

}
