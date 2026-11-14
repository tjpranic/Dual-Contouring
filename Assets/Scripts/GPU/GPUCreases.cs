using System;
using System.Runtime.InteropServices;
using UnityEngine;

using SmoothNormals = SurfaceExtractor.SmoothNormals;

// creases the contour a GPU implementation leaves in its buffers, on the GPU, by the rule of BurstCreases
// (CreaseNormals.compute). The contour's vertices and normals stay as they are, its quads are rewritten in place to the
// creased vertices, and the creased vertices, normals and counts are in buffers of this one's own, which it keeps from
// one crease to the next and makes larger when a contour needs more
public class GPUCreases {

    private readonly ComputeShader shader;

    private readonly int clearVerticesKernel;
    private readonly int triangleFacesKernel;
    private readonly int countCornersKernel;
    private readonly int placeRunsKernel;
    private readonly int scatterCornersKernel;
    private readonly int groupCornersKernel;
    private readonly int rewriteKernel;
    private readonly int finishKernel;

    private ComputeBuffer facesBuffer;
    private ComputeBuffer weightsBuffer;
    private ComputeBuffer cornerCountsBuffer;
    private ComputeBuffer cursorsBuffer;
    private ComputeBuffer runStartsBuffer;
    private ComputeBuffer runsBuffer;
    private ComputeBuffer sumsBuffer;
    private ComputeBuffer groupOfCornersBuffer;
    private ComputeBuffer basesBuffer;
    private ComputeBuffer totalsBuffer;
    private ComputeBuffer creasedVerticesBuffer;
    private ComputeBuffer creasedNormalsBuffer;
    private ComputeBuffer creasedCountsBuffer;

    // the most thread groups a dispatch takes along x, past which it goes on along y: 65,535, the most a dispatch may
    // take along an axis. Tests make it small to send the dispatches of small meshes along y
    public static int groupsAcross = 65535;

    private const int threadsPerGroup = 64;

    public GPUCreases( ) {
        this.shader = Resources.Load<ComputeShader>( "Shaders/CreaseNormals" );

        this.clearVerticesKernel  = this.shader.FindKernel( "clearVertices" );
        this.triangleFacesKernel  = this.shader.FindKernel( "triangleFaces" );
        this.countCornersKernel   = this.shader.FindKernel( "countCorners" );
        this.placeRunsKernel      = this.shader.FindKernel( "placeRuns" );
        this.scatterCornersKernel = this.shader.FindKernel( "scatterCorners" );
        this.groupCornersKernel   = this.shader.FindKernel( "groupCorners" );
        this.rewriteKernel        = this.shader.FindKernel( "rewrite" );
        this.finishKernel         = this.shader.FindKernel( "finish" );
    }

    // a buffer with room for at least that many elements: the one there is if it is large enough, a larger one
    // otherwise, half as much again. A compute buffer holds at least one element
    private static void reserve( ref ComputeBuffer buffer, int count, int stride ) {
        if( buffer != null && buffer.count >= count ) {
            return;
        }

        buffer?.Release( );
        buffer = new ComputeBuffer( Mathf.Max( 1, count + ( count / 2 ) ), stride );
    }

    // a dispatch of a thread for each of so many things, in groups of 64, going on along y past 65,535 groups. The
    // kernels stop the threads past the counts of the contour
    private void dispatch( int kernel, int threads ) {
        var groups = Mathf.Max( 1, ( threads + threadsPerGroup - 1 ) / threadsPerGroup );
        var across = Mathf.Min( groups, groupsAcross );

        this.shader.SetInt( "groupsX", across );
        this.shader.Dispatch( kernel, across, ( groups + across - 1 ) / across, 1 );
    }

    // the cosine the crease angle is held to, as BurstCreases has it
    public static float cosineOf( float angle ) {
        return angle >= 180.0f ? -2.0f : ( float )Math.Cos( Math.Max( angle, 0.0f ) * Math.PI / 180.0 );
    }

    // creases the contour in the buffers, which has at most so many vertices and quads: counts[0] and counts[1] say how
    // many it has. The angle is in degrees
    public void crease(
        ComputeBuffer     vertices,
        ComputeBuffer     normals,
        ComputeBuffer     quads,
        ComputeBuffer     counts,
        int               vertexCapacity,
        int               quadCapacity,
        float             angle,
        SmoothNormals     smoothNormals,
        out ComputeBuffer creasedVertices,
        out ComputeBuffer creasedNormals,
        out ComputeBuffer creasedCounts
    ) {
        var triangleCapacity = quadCapacity * 2;
        var cornerCapacity   = quadCapacity * 6;

        // every vertex once for each of its groups, and a vertex no triangle uses once
        var creasedCapacity = vertexCapacity + cornerCapacity;

        reserve( ref this.facesBuffer,           triangleCapacity, Marshal.SizeOf( typeof( Vector3 ) ) );
        reserve( ref this.weightsBuffer,         cornerCapacity,   sizeof( float ) );
        reserve( ref this.cornerCountsBuffer,    vertexCapacity,   sizeof( uint ) );
        reserve( ref this.cursorsBuffer,         vertexCapacity,   sizeof( uint ) );
        reserve( ref this.runStartsBuffer,       vertexCapacity,   sizeof( uint ) );
        reserve( ref this.runsBuffer,            cornerCapacity,   sizeof( uint ) );
        reserve( ref this.sumsBuffer,            cornerCapacity,   Marshal.SizeOf( typeof( Vector3 ) ) );
        reserve( ref this.groupOfCornersBuffer,  cornerCapacity,   sizeof( uint ) );
        reserve( ref this.basesBuffer,           vertexCapacity,   sizeof( uint ) );
        reserve( ref this.totalsBuffer,          2,                sizeof( uint ) );
        reserve( ref this.creasedVerticesBuffer, creasedCapacity,  Marshal.SizeOf( typeof( Vector3 ) ) );
        reserve( ref this.creasedNormalsBuffer,  creasedCapacity,  Marshal.SizeOf( typeof( Vector3 ) ) );
        reserve( ref this.creasedCountsBuffer,   2,                sizeof( uint ) );

        this.totalsBuffer.SetData( new uint[] { 0, 0 } );

        this.shader.SetFloat( "cosine",        cosineOf( angle ) );
        this.shader.SetInt  ( "smoothNormals", ( int )smoothNormals );

        foreach( var kernel in new[] { this.clearVerticesKernel, this.triangleFacesKernel, this.countCornersKernel, this.placeRunsKernel, this.scatterCornersKernel, this.groupCornersKernel, this.rewriteKernel, this.finishKernel } ) {
            this.shader.SetBuffer( kernel, "vertices",        vertices );
            this.shader.SetBuffer( kernel, "normals",         normals );
            this.shader.SetBuffer( kernel, "counts",          counts );
            this.shader.SetBuffer( kernel, "quads",           quads );
            this.shader.SetBuffer( kernel, "faces",           this.facesBuffer );
            this.shader.SetBuffer( kernel, "weights",         this.weightsBuffer );
            this.shader.SetBuffer( kernel, "cornerCounts",    this.cornerCountsBuffer );
            this.shader.SetBuffer( kernel, "cursors",         this.cursorsBuffer );
            this.shader.SetBuffer( kernel, "runStarts",       this.runStartsBuffer );
            this.shader.SetBuffer( kernel, "runs",            this.runsBuffer );
            this.shader.SetBuffer( kernel, "sums",            this.sumsBuffer );
            this.shader.SetBuffer( kernel, "groupOfCorners",  this.groupOfCornersBuffer );
            this.shader.SetBuffer( kernel, "bases",           this.basesBuffer );
            this.shader.SetBuffer( kernel, "totals",          this.totalsBuffer );
            this.shader.SetBuffer( kernel, "creasedVertices", this.creasedVerticesBuffer );
            this.shader.SetBuffer( kernel, "creasedNormals",  this.creasedNormalsBuffer );
            this.shader.SetBuffer( kernel, "creasedCounts",   this.creasedCountsBuffer );
        }

        // the read-only views groupCorners reads through
        this.shader.SetBuffer( this.groupCornersKernel, "facesRead",        this.facesBuffer );
        this.shader.SetBuffer( this.groupCornersKernel, "weightsRead",      this.weightsBuffer );
        this.shader.SetBuffer( this.groupCornersKernel, "cornerCountsRead", this.cornerCountsBuffer );
        this.shader.SetBuffer( this.groupCornersKernel, "runStartsRead",    this.runStartsBuffer );

        this.dispatch( this.clearVerticesKernel,  vertexCapacity );
        this.dispatch( this.triangleFacesKernel,  triangleCapacity );
        this.dispatch( this.countCornersKernel,   cornerCapacity );
        this.dispatch( this.placeRunsKernel,      vertexCapacity );
        this.dispatch( this.scatterCornersKernel, cornerCapacity );
        this.dispatch( this.groupCornersKernel,   vertexCapacity );
        this.dispatch( this.rewriteKernel,        quadCapacity );

        this.shader.Dispatch( this.finishKernel, 1, 1, 1 );

        creasedVertices = this.creasedVerticesBuffer;
        creasedNormals  = this.creasedNormalsBuffer;
        creasedCounts   = this.creasedCountsBuffer;
    }

    // the next crease makes them all anew
    public void release( ) {
        this.facesBuffer?.Release( );
        this.weightsBuffer?.Release( );
        this.cornerCountsBuffer?.Release( );
        this.cursorsBuffer?.Release( );
        this.runStartsBuffer?.Release( );
        this.runsBuffer?.Release( );
        this.sumsBuffer?.Release( );
        this.groupOfCornersBuffer?.Release( );
        this.basesBuffer?.Release( );
        this.totalsBuffer?.Release( );
        this.creasedVerticesBuffer?.Release( );
        this.creasedNormalsBuffer?.Release( );
        this.creasedCountsBuffer?.Release( );

        this.facesBuffer           = null;
        this.weightsBuffer         = null;
        this.cornerCountsBuffer    = null;
        this.cursorsBuffer         = null;
        this.runStartsBuffer       = null;
        this.runsBuffer            = null;
        this.sumsBuffer            = null;
        this.groupOfCornersBuffer  = null;
        this.basesBuffer           = null;
        this.totalsBuffer          = null;
        this.creasedVerticesBuffer = null;
        this.creasedNormalsBuffer  = null;
        this.creasedCountsBuffer   = null;
    }

}
