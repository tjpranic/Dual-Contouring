using System.Collections.Generic;
using UnityEngine;

public class Cell {

    public enum Sign {
        None,
        Inside,
        Outside
    }

    public class Corner {

        public Vector3 position;

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

        public ( Corner, Corner ) corners;

        public Vector3 intersection;
        public Vector3 normal;

        public Edge( ( Corner, Corner ) corners ) {
            this.corners = corners;
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

    public Vector3 minimum { get; private set; }
    public Vector3 maximum { get; private set; }

    private Corner[] _corners = new Corner[8];
    public Corner[] corners {
        get { return this._corners;  }
        set { this._corners = value; }
    }

    private Edge[] _edges = new Edge[12];
    public Edge[] edges {
        get { return this._edges;  }
        set { this._edges = value; }
    }

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
            new Corner( this.minimum ),
            new Corner( new Vector3( this.maximum.x, this.minimum.y, this.minimum.z ) ),
            new Corner( new Vector3( this.maximum.x, this.minimum.y, this.maximum.z ) ),
            new Corner( new Vector3( this.minimum.x, this.minimum.y, this.maximum.z ) ),
            // top
            new Corner( new Vector3( this.maximum.x, this.maximum.y, this.minimum.z ) ),
            new Corner( new Vector3( this.minimum.x, this.maximum.y, this.minimum.z ) ),
            new Corner( new Vector3( this.minimum.x, this.maximum.y, this.maximum.z ) ),
            new Corner( this.maximum ),
        };

        this.edges = new Edge[] {
            // x axis
            new Edge( ( this.corners[0], this.corners[1] ) ),
            new Edge( ( this.corners[3], this.corners[2] ) ),
            new Edge( ( this.corners[5], this.corners[4] ) ),
            new Edge( ( this.corners[6], this.corners[7] ) ),
            // y axis
            /*
            new Edge( ( this.corners[0], this.corners[5] ) ),
            new Edge( ( this.corners[1], this.corners[4] ) ),
            new Edge( ( this.corners[3], this.corners[6] ) ),
            new Edge( ( this.corners[2], this.corners[7] ) ),
            */
            new Edge( ( this.corners[5], this.corners[0] ) ),
            new Edge( ( this.corners[4], this.corners[1] ) ),
            new Edge( ( this.corners[6], this.corners[3] ) ),
            new Edge( ( this.corners[7], this.corners[2] ) ),
            // z axis
            new Edge( ( this.corners[0], this.corners[3] ) ),
            new Edge( ( this.corners[1], this.corners[2] ) ),
            new Edge( ( this.corners[5], this.corners[6] ) ),
            new Edge( ( this.corners[4], this.corners[7] ) )
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