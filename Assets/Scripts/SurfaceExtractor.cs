using System.Collections.Generic;
using UnityEngine;

public class Cell {

    public enum Sign {
        None,
        Inside,
        Outside
    }

    public class Corner {

        public readonly Vector3 position;

        private float _density;
        public float density {
            get { return this._density; }
            set {
                this._density = value;
                this.sign     = value < 0.0f ? Sign.Inside : Sign.Outside;
            }
        }

        public Sign sign { get; private set; }

        public Corner( Vector3 position ) {
            this.position = position;
            this.density  = 0.0f;
            this.sign     = Sign.None;
        }

    }

    public class Edge {

        public readonly Corner[] corners = new Corner[2];

        public Vector3 intersection;
        public Vector3 normal;

        public Edge( Corner[] corners ) {
            this.corners = corners;
        }

    }

    public class Adjacency {

        public readonly Vector3Int[] offsets;
        public readonly int          edgeIndex;

        public Adjacency( Vector3Int[] offsets, int edgeIndex ) {
            this.offsets   = offsets;
            this.edgeIndex = edgeIndex;
        }

    }

    private Vector3 _center;
    public Vector3 center {
        get { return this._center; }
        set {
            this._center = value;
            this.recalculate( );
        }
    }

    private Vector3 _size;
    public Vector3 size {
        get { return this._size; }
        set {
            this._size    = value;
            this._extents = value / 2;
            this.recalculate( );
        }
    }

    private Vector3 _extents;
    public Vector3 extents {
        get { return this._extents; }
        set {
            this._extents = value;
            this._size    = value * 2;
            this.recalculate( );
        }
    }

    public Vector3  minimum { get; private set; }
    public Vector3  maximum { get; private set; }
    public Corner[] corners { get; private set; } = new Corner[8];
    public Edge[]   edges   { get; private set; } = new Edge[12];

    public Vector3 vertex;
    public Vector3 normal;
    public int     index = -1;

    public Cell( Vector3 center, Vector3 size ) {
        this.center  = center;
        this.size    = size;
        this.extents = size / 2;

        this.recalculate( );
    }

    private void recalculate( ) {
        this.minimum = this.center - this.extents;
        this.maximum = this.center + this.extents;

        // sanity checking against unity implementation
        var bounds = new Bounds( this.center, this.size );
        Debug.Assert( this.minimum == bounds.min );
        Debug.Assert( this.maximum == bounds.max );

        // NOTE: do not change the order of theses arrays as lookup tables are order dependent

        this.corners = new Corner[] {
            // bottom
            new( this.minimum ),
            new( new Vector3( this.maximum.x, this.minimum.y, this.minimum.z ) ),
            new( new Vector3( this.maximum.x, this.minimum.y, this.maximum.z ) ),
            new( new Vector3( this.minimum.x, this.minimum.y, this.maximum.z ) ),
            // top
            new( new Vector3( this.maximum.x, this.maximum.y, this.minimum.z ) ),
            new( new Vector3( this.minimum.x, this.maximum.y, this.minimum.z ) ),
            new( new Vector3( this.minimum.x, this.maximum.y, this.maximum.z ) ),
            new( this.maximum ),
        };

        this.edges = new Edge[] {
            // x axis
            new( new Corner[] { this.corners[0], this.corners[1] } ),
            new( new Corner[] { this.corners[3], this.corners[2] } ),
            new( new Corner[] { this.corners[5], this.corners[4] } ),
            new( new Corner[] { this.corners[6], this.corners[7] } ),
            // y axis
            new( new Corner[] { this.corners[5], this.corners[0] } ),
            new( new Corner[] { this.corners[4], this.corners[1] } ),
            new( new Corner[] { this.corners[6], this.corners[3] } ),
            new( new Corner[] { this.corners[7], this.corners[2] } ),
            // z axis
            new( new Corner[] { this.corners[0], this.corners[3] } ),
            new( new Corner[] { this.corners[1], this.corners[2] } ),
            new( new Corner[] { this.corners[5], this.corners[6] } ),
            new( new Corner[] { this.corners[4], this.corners[7] } )
        };
    }

    public Bounds toBounds( ) {
        return new Bounds( this.center, this.size );
    }

}

public interface SurfaceExtractor {

    public ( Vector3[] positions, Vector3[] normals, int[] indices ) voxelize( DensityFunction densityFunction, int resolution );

    public List<Cell> getGridCells( );

}