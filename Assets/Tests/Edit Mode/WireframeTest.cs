using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

using Object             = UnityEngine.Object;
using Combination        = DensityFunction.Combination;
using MaterialIndex      = SurfaceExtractor.MaterialIndex;
using ImplementationType = SurfaceExtractor.Implementation.Type;
using QEFSolverType      = QEFSolver.Type;
using VertexNormals      = SurfaceExtractor.VertexNormals;
using Look               = Wireframe.Look;

// the wireframe drawn over a contour, seen through a camera of its own: where the lines go and what colour they are,
// which lines are hidden, and that the layout of the mesh makes no difference. The contours are drawn by a mesh
// renderer with unlit materials, and the wireframe over them, as the voxelizer has it. Then the voxelizer's own
// drawing: that it draws the lines of the contour it shows, and nothing else, whether it shows a mesh or draws the
// contour straight from the GPU
public class WireframeTest : ContouringTest {

    private const int size = 512;

    // at 3 pixels, the pixel that holds a point of an edge is wholly line, whichever way the edge runs
    private const float lineWidth = 3.0f;

    // a cube in the middle of the grid, its front face square to the camera. No face is on a plane of the grid
    private static readonly Shape[] cubeModel = { cube( Vector3.zero, 0.3f ) };

    // a cube of the first material in front, and one of the second material behind it, about half hidden by it. The
    // camera looks at both from inside their outlines, so of each only the front face faces it. Their faces are not on
    // planes of the grid, and the cubes are drawn in perspective: the vertices of both front faces are at the centres
    // of their cells, and seen straight on, the lines of the cube behind would lie on those of the cube in front
    private static readonly Shape[] twoCubes = {
        cube( new Vector3( -0.12f, -0.02f, -0.20f ), 0.17f, Combination.Union, MaterialIndex.Material0 ),
        cube( new Vector3(  0.06f,  0.07f,  0.20f ), 0.17f, Combination.Union, MaterialIndex.Material1 )
    };

    // colours that are neither 0 nor 1 in some channel, so a colour taken to linear space twice or not at all shows
    private static readonly Color   lineColor      = new( 1.0f, 0.5f, 0.0f, 1.0f );
    private static readonly Color   fillColor      = new( 0.5f, 0.0f, 0.5f, 1.0f );
    private static readonly Color[] materialColors = { new( 0.25f, 0.5f, 1.0f, 1.0f ), new( 1.0f, 0.75f, 0.25f, 1.0f ) };

    private readonly List<Object> made = new( );

    private bool            asyncCompilation;
    private Wireframe       wireframe;
    private OffscreenRender offscreen;
    private GameObject      shown;
    private Mesh            drawn;
    private Material[]      drawnMaterials;
    private Voxelizer       drawnBy;
    private Look            look;

    [SetUp]
    public void setUp( ) {
        // a shader still compiling in the background draws as a placeholder
        this.asyncCompilation = ShaderUtil.allowAsyncCompilation;
        ShaderUtil.allowAsyncCompilation = false;

        this.wireframe = new Wireframe( );

        RenderPipelineManager.beginCameraRendering += this.drawWireframe;
    }

    [TearDown]
    public void tearDownWireframe( ) {
        RenderPipelineManager.beginCameraRendering -= this.drawWireframe;

        this.wireframe.Dispose( );
        this.offscreen?.Dispose( );

        foreach( var item in this.made ) {
            Object.DestroyImmediate( item );
        }
        this.made.Clear( );

        ShaderUtil.allowAsyncCompilation = this.asyncCompilation;
    }

    // draws the mesh with the test's own Wireframe, or has the voxelizer draw its wireframe, as it does for every camera
    // in play mode
    private void drawWireframe( ScriptableRenderContext context, Camera camera ) {
        if( this.offscreen == null || camera != this.offscreen.camera ) {
            return;
        }

        if( this.drawnBy != null ) {
            this.drawnBy.DrawShapes( camera );
            return;
        }

        if( this.drawn == null ) {
            return;
        }

        var style = new Wireframe.Style {
            look      = this.look,
            lineWidth = lineWidth,
            lineColor = lineColor,
            fillColor = fillColor
        };

        this.wireframe.draw( camera, this.drawn, Matrix4x4.identity, OffscreenRender.layer, style, this.drawnMaterials );
    }

    private void view( bool perspective ) {
        this.offscreen = new OffscreenRender( size, 1.0f, perspective );
    }

    // the contour of the shapes on 16^3 cells, from Burst: positions and normals in streams of their own, 16-bit indices
    private Mesh contour( Shape[] shapes ) {
        return this.voxelize( this.createUniformVoxelizer( 4, ImplementationType.Burst, QEFSolverType.SVD, shapes ) ).mesh;
    }

    // a mesh renderer for the mesh, in place of the one before, with an unlit material of each colour. The wireframe is
    // drawn over it
    private void show( Mesh mesh, params Color[] colors ) {
        if( this.shown != null ) {
            this.made.Remove( this.shown );
            Object.DestroyImmediate( this.shown );
        }

        this.shown = new GameObject( "Contour" ) { hideFlags = HideFlags.HideAndDontSave, layer = OffscreenRender.layer };
        this.made.Add( this.shown );

        this.shown.AddComponent<MeshFilter>( ).sharedMesh = mesh;

        var materials = this.unlitMaterials( colors );

        this.shown.AddComponent<MeshRenderer>( ).sharedMaterials = materials;

        this.drawn          = mesh;
        this.drawnMaterials = materials;
    }

    private Material[] unlitMaterials( Color[] colors ) {
        return colors.Select(
            ( color ) => {
                var material = Object.Instantiate( Resources.Load<Material>( "Materials/unlit_white" ) );
                material.hideFlags = HideFlags.HideAndDontSave;
                material.color     = color;
                this.made.Add( material );
                return material;
            }
        ).ToArray( );
    }

    // a voxelizer that contours the shapes on 16^3 cells with Burst and shows the contour with an unlit material of each
    // colour. It draws its own wireframe, with the tests' lines
    private Voxelizer voxelizerShowing( Shape[] shapes, params Color[] colors ) {
        var voxelizer = this.createUniformVoxelizer( 4, ImplementationType.Burst, QEFSolverType.SVD, shapes );

        voxelizer.gameObject.layer = OffscreenRender.layer;
        voxelizer.GetComponent<MeshRenderer>( ).sharedMaterials = this.unlitMaterials( colors );

        voxelizer.wireframeLineWidth = lineWidth;
        voxelizer.wireframeLineColor = lineColor;
        voxelizer.wireframeFillColor = fillColor;

        voxelizer.contour( drawFromGPU: false );

        this.drawnBy = voxelizer;

        return voxelizer;
    }

    public enum GPUAlgorithm {
        Uniform,
        Occupancy
    }

    // a voxelizer whose contour the GPU makes and draws straight from its buffers, with an unlit material of each
    // colour. It draws its own wireframe, with the tests' lines
    private Voxelizer gpuVoxelizerShowing( GPUAlgorithm algorithm, VertexNormals normals, Shape[] shapes, params Color[] colors ) {
        Voxelizer voxelizer = algorithm == GPUAlgorithm.Uniform
            ? this.createVoxelizer<UniformDualContouring>( 4, QEFSolverType.SVD, shapes )
            : this.createVoxelizer<OccupancyDualContouring>( 4, QEFSolverType.SVD, shapes );

        voxelizer.implementationType = ImplementationType.GPU;
        voxelizer.vertexNormals      = normals;

        // creased on the GPU, when creased, as a voxelizer starts out
        voxelizer.creaseAngle = 40.0f;

        voxelizer.gameObject.layer = OffscreenRender.layer;
        voxelizer.GetComponent<MeshRenderer>( ).sharedMaterials = this.unlitMaterials( colors );

        voxelizer.wireframeLineWidth = lineWidth;
        voxelizer.wireframeLineColor = lineColor;
        voxelizer.wireframeFillColor = fillColor;

        voxelizer.contour( drawFromGPU: true );

        var meshFilter = voxelizer.GetComponent<MeshFilter>( );
        Assert.IsTrue( meshFilter == null || meshFilter.sharedMesh == null, "drawn straight from the GPU, with nothing in the mesh filter" );

        this.drawnBy = voxelizer;

        return voxelizer;
    }

    // the GPU's contour read back, in a mesh renderer. Update draws the GPU's contour itself, and only in play mode, so
    // this gives the surface, and the depth, that the GPU's lines go over. Contouring again fills the same buffers the
    // voxelizer draws from, and the GPU's vertices are the same each time
    private Mesh showReadBack( Voxelizer voxelizer, params Color[] colors ) {
        var readBack = this.voxelize( voxelizer ).mesh;

        var drawnBy = this.drawnBy;
        this.show( readBack, colors );
        this.drawnBy = drawnBy;

        return readBack;
    }

    // the mesh the voxelizer shows, drawn with the test's own Wireframe instead of by the voxelizer
    private Color32[] renderByTheTest( Voxelizer voxelizer, Look look ) {
        var drawnBy = this.drawnBy;

        this.drawnBy        = null;
        this.drawn          = voxelizer.GetComponent<MeshFilter>( ).sharedMesh;
        this.drawnMaterials = voxelizer.GetComponent<MeshRenderer>( ).sharedMaterials;

        var image = this.render( look );

        this.drawnBy = drawnBy;
        this.drawn   = null;

        return image;
    }

    // what the camera sees with no wireframe drawn at all
    private Color32[] renderWithout( ) {
        var ( drawn, drawnBy ) = ( this.drawn, this.drawnBy );

        this.drawn   = null;
        this.drawnBy = null;

        var image = this.offscreen.render( );

        ( this.drawn, this.drawnBy ) = ( drawn, drawnBy );

        return image;
    }

    private Color32[] render( Look look ) {
        this.look = look;

        if( this.drawnBy != null ) {
            this.drawnBy.wireframe = look;
        }

        return this.offscreen.render( );
    }

    // a triangle of the mesh that faces the camera squarely, in pixels
    private readonly struct Facing {

        public readonly Vector2 a;
        public readonly Vector2 b;
        public readonly Vector2 c;
        public readonly int     subMesh;

        public Facing( Vector2 a, Vector2 b, Vector2 c, int subMesh ) {
            this.a       = a;
            this.b       = b;
            this.c       = c;
            this.subMesh = subMesh;
        }

        public Vector2 centre => ( this.a + this.b + this.c ) / 3.0f;

        private static float cross( Vector2 u, Vector2 v ) {
            return ( u.x * v.y ) - ( u.y * v.x );
        }

        // the smallest of its heights, across from each corner
        public float height {
            get {
                var area    = Mathf.Abs( cross( this.b - this.a, this.c - this.a ) );
                var longest = Mathf.Max( ( this.b - this.a ).magnitude, Mathf.Max( ( this.c - this.b ).magnitude, ( this.a - this.c ).magnitude ) );
                return area / longest;
            }
        }

        public bool contains( Vector2 point ) {
            var first  = cross( this.b - this.a, point - this.a );
            var second = cross( this.c - this.b, point - this.b );
            var third  = cross( this.a - this.c, point - this.c );
            return ( first >= 0 && second >= 0 && third >= 0 ) || ( first <= 0 && second <= 0 && third <= 0 );
        }

        public float distanceTo( Vector2 point ) {
            return Mathf.Min( distance( point, this.a, this.b ), Mathf.Min( distance( point, this.b, this.c ), distance( point, this.c, this.a ) ) );
        }

        private static float distance( Vector2 point, Vector2 start, Vector2 end ) {
            var along = Mathf.Clamp01( Vector2.Dot( point - start, end - start ) / ( end - start ).sqrMagnitude );
            return ( point - ( start + ( ( end - start ) * along ) ) ).magnitude;
        }

    }

    // the triangles whose normal points at the camera, within about 8°
    private List<Facing> facing( Mesh mesh ) {
        var vertices = mesh.vertices;
        var toCamera = -this.offscreen.camera.transform.forward;
        var result   = new List<Facing>( );

        for( var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh ) {
            var triangles = mesh.GetTriangles( subMesh );

            for( var index = 0; index < triangles.Length; index += 3 ) {
                var a = vertices[triangles[index + 0]];
                var b = vertices[triangles[index + 1]];
                var c = vertices[triangles[index + 2]];

                if( Vector3.Dot( Vector3.Cross( b - a, c - a ).normalized, toCamera ) > 0.99f ) {
                    result.Add( new( this.offscreen.pixelOf( a ), this.offscreen.pixelOf( b ), this.offscreen.pixelOf( c ), subMesh ) );
                }
            }
        }

        return result;
    }

    // points a quarter, half and three quarters along the edges that two of the triangles share. An edge on the outline
    // of the triangles may be half outside the surface, so those are left out
    private static List<Vector2> sharedEdgePoints( IEnumerable<Facing> triangles ) {
        Vector2Int key( Vector2 point ) {
            return new( Mathf.RoundToInt( point.x * 1000.0f ), Mathf.RoundToInt( point.y * 1000.0f ) );
        }

        var counts = new Dictionary<( Vector2Int, Vector2Int ), ( Vector2 start, Vector2 end, int count )>( );

        void add( Vector2 start, Vector2 end ) {
            var ( first, second ) = ( key( start ), key( end ) );
            var edge = first.x < second.x || ( first.x == second.x && first.y < second.y ) ? ( first, second ) : ( second, first );
            counts[edge] = counts.TryGetValue( edge, out var known ) ? ( known.start, known.end, known.count + 1 ) : ( start, end, 1 );
        }

        foreach( var triangle in triangles ) {
            add( triangle.a, triangle.b );
            add( triangle.b, triangle.c );
            add( triangle.c, triangle.a );
        }

        return counts.Values
            .Where( ( edge ) => edge.count == 2 )
            .SelectMany( ( edge ) => new[] { 0.25f, 0.5f, 0.75f }.Select( ( along ) => Vector2.Lerp( edge.start, edge.end, along ) ) )
            .ToList( );
    }

    // the centres of the triangles that are large enough for their centre to be clear of the lines: further from every
    // edge than half the line, the pixel's own reach and what fwidth's overestimate of the gradient adds
    private static List<Vector2> clearCentres( IEnumerable<Facing> triangles ) {
        return triangles.Where( ( triangle ) => triangle.height > 12.0f ).Select( ( triangle ) => triangle.centre ).ToList( );
    }

    private static bool near( Color expected, Color32 actual ) {
        Color32 wanted = expected;
        return Mathf.Abs( wanted.r - actual.r ) <= 2 && Mathf.Abs( wanted.g - actual.g ) <= 2 && Mathf.Abs( wanted.b - actual.b ) <= 2;
    }

    // every point is the expected colour in the image, within rounding
    private void assertColors( Color32[] image, IEnumerable<Vector2> points, System.Func<Vector2, Color32> expected, string what ) {
        var wrong = points.Where( ( point ) => !near( expected( point ), this.offscreen.at( image, point ) ) ).ToList( );

        Assert.IsEmpty(
            wrong,
            $"{wrong.Count} {what} are the wrong colour, for example " +
            string.Join( ", ", wrong.Take( 4 ).Select( ( point ) => $"{point}: {this.offscreen.at( image, point )} instead of {expected( point )}" ) )
        );
    }

    private static void assertSameImage( Color32[] expected, Color32[] actual, string message ) {
        Assert.AreEqual( expected.Length, actual.Length );

        var differences = 0;
        var first       = -1;

        for( var index = 0; index < expected.Length; ++index ) {
            if( expected[index].r != actual[index].r || expected[index].g != actual[index].g || expected[index].b != actual[index].b ) {
                if( differences++ == 0 ) {
                    first = index;
                }
            }
        }

        Assert.AreEqual( 0, differences, $"{message}: {differences} pixels differ, the first at ({first % size}, {first / size})" );
    }

    // a mesh with the given layout and index format, and the same vertices and triangles
    private Mesh meshOf( bool twoStreams, IndexFormat format, Vector3[] vertices, Vector3[] normals, int[][] subMeshes ) {
        var mesh = new Mesh { hideFlags = HideFlags.HideAndDontSave };
        this.made.Add( mesh );

        if( !twoStreams ) {
            mesh.indexFormat  = format;
            mesh.vertices     = vertices;
            mesh.normals      = normals;
            mesh.subMeshCount = subMeshes.Length;

            for( var subMesh = 0; subMesh < subMeshes.Length; ++subMesh ) {
                mesh.SetTriangles( subMeshes[subMesh], subMesh );
            }
        }
        else {
            mesh.SetVertexBufferParams(
                vertices.Length,
                new VertexAttributeDescriptor( VertexAttribute.Position, VertexAttributeFormat.Float32, 3, 0 ),
                new VertexAttributeDescriptor( VertexAttribute.Normal,   VertexAttributeFormat.Float32, 3, 1 )
            );
            mesh.SetVertexBufferData( vertices, 0, 0, vertices.Length, 0 );
            mesh.SetVertexBufferData( normals,  0, 0, vertices.Length, 1 );

            var indices = subMeshes.SelectMany( ( triangles ) => triangles ).ToArray( );

            mesh.SetIndexBufferParams( indices.Length, format );

            if( format == IndexFormat.UInt32 ) {
                mesh.SetIndexBufferData( indices, 0, 0, indices.Length );
            }
            else {
                mesh.SetIndexBufferData( indices.Select( ( index ) => ( ushort )index ).ToArray( ), 0, 0, indices.Length );
            }

            mesh.subMeshCount = subMeshes.Length;

            var start = 0;
            for( var subMesh = 0; subMesh < subMeshes.Length; ++subMesh ) {
                mesh.SetSubMesh( subMesh, new SubMeshDescriptor( start, subMeshes[subMesh].Length ) );
                start += subMeshes[subMesh].Length;
            }
        }

        mesh.RecalculateBounds( );

        Assert.AreEqual( twoStreams ? 1 : 0, mesh.GetVertexAttributeStream( VertexAttribute.Normal ), "the stream of the normals" );
        Assert.AreEqual( format, mesh.indexFormat );

        return mesh;
    }

    [Test]
    public void offDrawsNothing( ) {
        this.view( perspective: false );
        this.show( this.contour( cubeModel ), materialColors[0] );

        var off = this.render( Look.Off );

        assertSameImage( this.renderWithout( ), off, "with the wireframe off, against no wireframe at all" );
    }

    [Test]
    public void overSurfaceDrawsTheEdgesOverTheSurface( ) {
        this.view( perspective: false );

        var mesh = this.contour( cubeModel );
        this.show( mesh, materialColors[0] );

        var surface = this.render( Look.Off );
        var over    = this.render( Look.OverSurface );

        var triangles = this.facing( mesh );
        var edges     = sharedEdgePoints( triangles );
        var centres   = clearCentres( triangles );

        Assert.Greater( edges.Count,   300, "points on edges inside the front face" );
        Assert.Greater( centres.Count, 80,  "triangles of the front face clear of the lines" );

        this.assertColors( over, edges,   ( point ) => lineColor,                             "points on edges" );
        this.assertColors( over, centres, ( point ) => this.offscreen.at( surface, point ), "triangle centres" );
    }

    [Test]
    public void linesOnlyDrawsTheEdgesOverTheFill( ) {
        this.view( perspective: false );

        var mesh = this.contour( cubeModel );
        this.show( mesh, materialColors[0] );

        var lines = this.render( Look.LinesOnly );

        var triangles = this.facing( mesh );
        var edges     = sharedEdgePoints( triangles );
        var centres   = clearCentres( triangles );

        Assert.Greater( edges.Count,   300, "points on edges inside the front face" );
        Assert.Greater( centres.Count, 80,  "triangles of the front face clear of the lines" );

        this.assertColors( lines, edges,   ( point ) => materialColors[0], "points on edges" );
        this.assertColors( lines, centres, ( point ) => fillColor,          "triangle centres" );
    }

    [Test]
    public void linesBehindTheSurfaceAreHidden( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: true );

        var mesh = this.contour( twoCubes );
        this.show( mesh, materialColors );

        var surface = this.render( Look.Off );
        var image   = this.render( look );

        var triangles = this.facing( mesh );
        var front     = triangles.Where( ( triangle ) => triangle.subMesh == 0 ).ToList( );
        var behind    = sharedEdgePoints( triangles.Where( ( triangle ) => triangle.subMesh == 1 ) );

        // clear of the front cube's lines and of its outline
        bool clear( Vector2 point ) => front.All( ( triangle ) => triangle.distanceTo( point ) > 4.0f );

        var hidden  = behind.Where( ( point ) =>  front.Any( ( triangle ) => triangle.contains( point ) ) && clear( point ) ).ToList( );
        var visible = behind.Where( ( point ) => !front.Any( ( triangle ) => triangle.contains( point ) ) && clear( point ) ).ToList( );

        Assert.Greater( hidden.Count,  15, "points on edges of the cube behind, hidden by the cube in front" );
        Assert.Greater( visible.Count, 50, "points on edges of the cube behind that can be seen" );

        if( look == Look.OverSurface ) {
            this.assertColors( image, hidden,  ( point ) => this.offscreen.at( surface, point ), "hidden edges" );
            this.assertColors( image, visible, ( point ) => lineColor,                           "edges in view" );
        }
        else {
            this.assertColors( image, hidden,  ( point ) => fillColor,         "hidden edges" );
            this.assertColors( image, visible, ( point ) => materialColors[1], "edges in view" );
        }
    }

    [Test]
    public void linesOnlyDrawsEachSubMeshInItsMaterialsColour( ) {
        this.view( perspective: true );

        var mesh = this.contour( twoCubes );
        this.show( mesh, materialColors );

        var lines = this.render( Look.LinesOnly );

        var triangles = this.facing( mesh );
        var front     = triangles.Where( ( triangle ) => triangle.subMesh == 0 ).ToList( );
        var behind    = sharedEdgePoints( triangles.Where( ( triangle ) => triangle.subMesh == 1 ) )
            .Where( ( point ) => !front.Any( ( triangle ) => triangle.contains( point ) ) && front.All( ( triangle ) => triangle.distanceTo( point ) > 4.0f ) )
            .ToList( );

        var inFront = sharedEdgePoints( front );

        Assert.Greater( inFront.Count, 100, "points on edges of the cube in front" );
        Assert.Greater( behind.Count,  50,  "points on edges of the cube behind that can be seen" );

        this.assertColors( lines, inFront, ( point ) => materialColors[0], "edges of the first material" );
        this.assertColors( lines, behind,  ( point ) => materialColors[1], "edges of the second material" );
    }

    [Test]
    public void subMeshWithoutAMaterialHasNoLines( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: true );

        // the renderer has a material for the cube in front only, so it does not draw the cube behind
        var mesh = this.contour( twoCubes );
        this.show( mesh, materialColors[0] );

        var image = this.render( look );

        var triangles = this.facing( mesh );
        var front     = triangles.Where( ( triangle ) => triangle.subMesh == 0 ).ToList( );
        var behind    = sharedEdgePoints( triangles.Where( ( triangle ) => triangle.subMesh == 1 ) )
            .Where( ( point ) => !front.Any( ( triangle ) => triangle.contains( point ) ) && front.All( ( triangle ) => triangle.distanceTo( point ) > 4.0f ) )
            .ToList( );

        Assert.Greater( behind.Count, 50, "points on edges of the cube behind, outside the cube in front" );

        this.assertColors( image, behind, ( point ) => Color.black, "edges of the sub-mesh without a material, against the background" );
    }

    [Test]
    public void everyLayoutAndIndexFormatGivesTheSameImage( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: true );

        var contoured = this.contour( twoCubes );

        Assert.AreEqual( 1,                  contoured.GetVertexAttributeStream( VertexAttribute.Normal ), "Burst's layout: the normals in a stream of their own" );
        Assert.AreEqual( IndexFormat.UInt16, contoured.indexFormat );

        var vertices  = contoured.vertices;
        var normals   = contoured.normals;
        var subMeshes = new[] { contoured.GetTriangles( 0 ), contoured.GetTriangles( 1 ) };

        // one more triangle in the first sub-mesh: a copy of one on the back of its cube, which faces away and is culled.
        // The first sub-mesh then has an odd number of indices, and the second starts halfway into a 32-bit word of
        // 16-bit indices
        var away = Enumerable.Range( 0, subMeshes[0].Length / 3 ).First(
            ( triangle ) => {
                var a = vertices[subMeshes[0][( triangle * 3 ) + 0]];
                var b = vertices[subMeshes[0][( triangle * 3 ) + 1]];
                var c = vertices[subMeshes[0][( triangle * 3 ) + 2]];
                return Vector3.Dot( Vector3.Cross( b - a, c - a ).normalized, Vector3.forward ) > 0.99f;
            }
        );
        subMeshes[0] = subMeshes[0].Concat( subMeshes[0].Skip( away * 3 ).Take( 3 ) ).ToArray( );

        Assert.AreEqual( 1, subMeshes[0].Length % 2, "an odd number of indices in the first sub-mesh" );

        this.show( contoured, materialColors );
        var expected = this.render( look );

        foreach( var twoStreams in new[] { false, true } ) {
            foreach( var format in new[] { IndexFormat.UInt16, IndexFormat.UInt32 } ) {
                this.show( this.meshOf( twoStreams, format, vertices, normals, subMeshes ), materialColors );
                assertSameImage( expected, this.render( look ), $"{( twoStreams ? "two streams" : "one stream" )}, {format}" );
            }
        }
    }

    [Test]
    public void splitVerticesGiveTheSameImage( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: true );

        var shared = this.contour( twoCubes );
        this.show( shared, materialColors );
        var expected = this.render( look );

        var split = Object.Instantiate( shared );
        split.hideFlags = HideFlags.HideAndDontSave;
        this.made.Add( split );

        Voxelizer.splitVertices( split );

        Assert.AreEqual( split.GetIndexCount( 0 ) + split.GetIndexCount( 1 ), ( uint )split.vertexCount, "three vertices of its own for every triangle" );

        this.show( split, materialColors );
        assertSameImage( expected, this.render( look ), "split vertices, against shared ones" );
    }

    [Test]
    public void drawingLeavesTheMeshAsItWas( ) {
        this.view( perspective: true );

        var mesh = this.contour( twoCubes );

        var vertices  = mesh.vertices;
        var normals   = mesh.normals;
        var format    = mesh.indexFormat;
        var subMeshes = Enumerable.Range( 0, mesh.subMeshCount ).Select( ( subMesh ) => mesh.GetTriangles( subMesh ) ).ToArray( );

        this.show( mesh, materialColors );
        this.render( Look.LinesOnly );
        this.render( Look.OverSurface );

        CollectionAssert.AreEqual( vertices, mesh.vertices );
        CollectionAssert.AreEqual( normals,  mesh.normals );
        Assert.AreEqual( format,           mesh.indexFormat );
        Assert.AreEqual( subMeshes.Length, mesh.subMeshCount );

        for( var subMesh = 0; subMesh < subMeshes.Length; ++subMesh ) {
            CollectionAssert.AreEqual( subMeshes[subMesh], mesh.GetTriangles( subMesh ), $"sub-mesh {subMesh}" );
        }
    }

    private static bool background( Color32[] image ) {
        return image.All( ( pixel ) => pixel.r == 0 && pixel.g == 0 && pixel.b == 0 );
    }

    // an empty contour, as of shapes that are all outside the grid, has a mesh without vertices, and such a mesh has no
    // buffers
    [Test]
    public void meshWithoutVerticesDrawsNothing( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: false );

        var empty = new Mesh { hideFlags = HideFlags.HideAndDontSave };
        this.made.Add( empty );

        this.show( empty, materialColors[0] );

        Assert.IsTrue( background( this.render( look ) ), "only the background" );
    }

    [Test]
    public void meshWithoutTrianglesDrawsNothing( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: false );

        var contoured = this.contour( cubeModel );
        var vertices  = contoured.vertices;

        this.show( this.meshOf( true, IndexFormat.UInt16, vertices, contoured.normals, new[] { new int[0] } ), materialColors[0] );

        Assert.IsTrue( background( this.render( look ) ), "only the background" );
    }

    [Test]
    public void voxelizerDrawsTheWireframeOverItsContour( [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: false );

        var voxelizer = this.voxelizerShowing( cubeModel, materialColors[0] );
        var image     = this.render( look );

        assertSameImage( this.renderByTheTest( voxelizer, look ), image, "drawn by the voxelizer, against the test's own drawing of its mesh" );

        var points = sharedEdgePoints( this.facing( voxelizer.GetComponent<MeshFilter>( ).sharedMesh ) );

        Assert.Greater( points.Count, 300, "points on edges inside the front face" );

        this.assertColors( image, points, ( point ) => look == Look.OverSurface ? lineColor : materialColors[0], "points on edges" );
    }

    [Test]
    public void voxelizerDrawsNothingWithTheWireframeOff( ) {
        this.view( perspective: false );

        this.voxelizerShowing( cubeModel, materialColors[0] );

        var off = this.render( Look.Off );

        assertSameImage( this.renderWithout( ), off, "with the wireframe off, against no wireframe at all" );
    }

    // the shapes are all outside the grid, so the contour is empty
    [Test]
    public void voxelizerWithAnEmptyContourDrawsNothing( ) {
        this.view( perspective: false );

        var voxelizer = this.voxelizerShowing( new[] { cube( new Vector3( 2.0f, 0.0f, 0.0f ), 0.1f ) }, materialColors[0] );

        Assert.AreEqual( 0, voxelizer.GetComponent<MeshFilter>( ).sharedMesh.vertexCount, "vertices of the contour" );

        Assert.IsTrue( background( this.render( Look.LinesOnly ) ), "only the background" );
    }

    // the mesh filter holds a mesh someone put there by hand, which the voxelizer has no lines for
    [Test]
    public void voxelizerDrawsNoWireframeOverAMeshItDidNotMake( ) {
        this.view( perspective: false );

        var voxelizer  = this.voxelizerShowing( cubeModel, materialColors[0] );
        var meshFilter = voxelizer.GetComponent<MeshFilter>( );

        var other = Object.Instantiate( meshFilter.sharedMesh );
        other.hideFlags = HideFlags.HideAndDontSave;
        this.made.Add( other );

        meshFilter.sharedMesh = other;

        var image = this.render( Look.LinesOnly );

        assertSameImage( this.renderWithout( ), image, "over a mesh the voxelizer did not make, against no wireframe at all" );
    }

    // the renderer draws no surface, so there is nothing to draw lines over
    [Test]
    public void voxelizerDrawsNoWireframeWhileItsRendererIsOff( ) {
        this.view( perspective: false );

        var voxelizer = this.voxelizerShowing( cubeModel, materialColors[0] );
        voxelizer.GetComponent<MeshRenderer>( ).enabled = false;

        var image = this.render( Look.LinesOnly );

        Assert.IsTrue( background( image ), "only the background" );
    }

    [Test]
    public void voxelizerOnALayerTheCameraDoesNotSeeDrawsNoWireframe( ) {
        this.view( perspective: false );

        var voxelizer = this.voxelizerShowing( cubeModel, materialColors[0] );
        voxelizer.gameObject.layer = 0;

        var image = this.render( Look.LinesOnly );

        Assert.IsTrue( background( image ), "only the background" );
    }

    // a contour shows a new mesh and destroys the one before, and the lines follow it
    [Test]
    public void voxelizerFollowsANewContour( ) {
        this.view( perspective: false );

        var voxelizer = this.voxelizerShowing( cubeModel, materialColors[0] );
        var before    = this.render( Look.LinesOnly );

        // still no face on a plane of the grid
        voxelizer.transform.GetChild( 0 ).localPosition = new Vector3( 0.1f, 0.04f, 0.0f );
        voxelizer.contour( drawFromGPU: false );

        var after = this.render( Look.LinesOnly );

        Assert.Greater( Enumerable.Range( 0, after.Length ).Count( ( index ) => !after[index].Equals( before[index] ) ), 1000, "pixels that changed with the cube" );

        assertSameImage( this.renderByTheTest( voxelizer, Look.LinesOnly ), after, "drawn by the voxelizer after the second contour, against the test's own drawing of the new mesh" );
    }

    // release lets go of the wireframe's material and the mesh's buffers, and the next drawing takes them again
    [Test]
    public void voxelizerDrawsTheSameWireframeAfterRelease( ) {
        this.view( perspective: false );

        var voxelizer = this.voxelizerShowing( cubeModel, materialColors[0] );
        var before    = this.render( Look.LinesOnly );
        var without   = this.renderWithout( );

        Assert.Greater( Enumerable.Range( 0, before.Length ).Count( ( index ) => !before[index].Equals( without[index] ) ), 1000, "pixels the wireframe drew" );

        voxelizer.release( );

        assertSameImage( before, this.render( Look.LinesOnly ), "after release, against before" );
    }

    // the GPU's quads, drawn by the shader's variant for them, get the same lines as the mesh read back from them
    [Test]
    public void gpuContourGetsTheLinesOfItsMesh( [Values] GPUAlgorithm algorithm, [Values] VertexNormals normals, [Values( Look.OverSurface, Look.LinesOnly )] Look look ) {
        this.view( perspective: true );

        var voxelizer = this.gpuVoxelizerShowing( algorithm, normals, twoCubes, materialColors );
        var readBack  = this.showReadBack( voxelizer, materialColors );

        var image = this.render( look );

        var without = this.renderWithout( );
        Assert.Greater( Enumerable.Range( 0, image.Length ).Count( ( index ) => !image[index].Equals( without[index] ) ), 1000, "pixels the wireframe drew" );

        // the same mesh, with the test's own Wireframe
        this.drawnBy = null;
        this.drawn   = readBack;

        assertSameImage( this.render( look ), image, "the GPU's lines, against those of the mesh read back from it" );
    }

    [Test]
    public void gpuContourWithTheWireframeOffDrawsNothing( ) {
        this.view( perspective: true );

        var voxelizer = this.gpuVoxelizerShowing( GPUAlgorithm.Uniform, VertexNormals.Creased, twoCubes, materialColors );
        this.showReadBack( voxelizer, materialColors );

        var off = this.render( Look.Off );

        assertSameImage( this.renderWithout( ), off, "with the wireframe off, against no wireframe at all" );
    }

    // release gives back the GPU's buffers, which the contour and its lines were drawn from. The lines are gone, and
    // nothing is logged
    [Test]
    public void gpuContourHasNoLinesAfterRelease( [Values] GPUAlgorithm algorithm ) {
        this.view( perspective: true );

        var voxelizer = this.gpuVoxelizerShowing( algorithm, VertexNormals.Creased, twoCubes, materialColors );
        this.showReadBack( voxelizer, materialColors );

        voxelizer.release( );

        var image = this.render( Look.LinesOnly );

        assertSameImage( this.renderWithout( ), image, "after release, against no wireframe at all" );
    }

}
