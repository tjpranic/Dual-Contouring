using System;
using System.Globalization;
using TMPro;
using UnityEngine;

[RequireComponent( typeof( TextMeshProUGUI ) )]
public class FPSCounter : MonoBehaviour {

    private static readonly FrameTiming[] latestTimings = new FrameTiming[1];

    public int   averageOver    = 60;   // frames
    public float updateInterval = 0.5f; // seconds

    private TMP_Text text;
    private float[]  frameBuffer    = Array.Empty<float>( );
    private float    currentFrameMs = 0.0f;
    private float    currentFPS     = 0.0f;
    private int      frameIndex     = 0;
    private int      sampledFrames  = 0;

    public void Start( ) {
        text           = GetComponent<TMP_Text>( );
        frameBuffer    = new float[Mathf.Max( 1, averageOver )];
        currentFrameMs = 0.0f;
        currentFPS     = 0.0f;
        frameIndex     = 0;
        sampledFrames  = 0;

        InvokeRepeating( nameof( GetFPS ), updateInterval, updateInterval );
    }

    public void Update( ) {
        FrameTimingManager.CaptureFrameTimings( );

        var frameTimeMs     = GetFrameTimeMilliseconds( );
        currentFrameMs = frameTimeMs;
        currentFPS     = frameTimeMs > 0.0f ? 1000.0f / frameTimeMs : 0.0f;

        frameBuffer[frameIndex] = frameTimeMs;
        frameIndex              = ( frameIndex + 1 ) % frameBuffer.Length;
        sampledFrames           = Mathf.Min( sampledFrames + 1, frameBuffer.Length );
    }

    private void GetFPS( ) {
        if( sampledFrames == 0 ) {
            return;
        }

        // summed fresh each time rather than kept as a running total, which drifts (even below zero) as rounding
        // errors from adding and removing frame times build up. slots not yet sampled are still zero
        var totalMs = 0.0f;

        foreach( var frameMs in frameBuffer ) {
            totalMs += frameMs;
        }

        var averageFrameTimeMs = totalMs / sampledFrames;
        var averageFrameRate   = averageFrameTimeMs > 0.0f ? 1000.0f / averageFrameTimeMs : 0.0f;

        text.text = string.Format(
            CultureInfo.InvariantCulture,
            "FPS: {0:0.0}\nFrame Time: {1:0.00} ms\nAverage Over ({2}) Frames: {3:0.0}",
            currentFPS,
            currentFrameMs,
            sampledFrames,
            averageFrameRate
        );
    }

    private float GetFrameTimeMilliseconds( ) {
        if( FrameTimingManager.GetLatestTimings( 1, latestTimings ) > 0 ) {
            var latestTiming = latestTimings[0];
            var frameTimeMs  = Mathf.Max( ( float ) latestTiming.cpuFrameTime, ( float ) latestTiming.gpuFrameTime );

            if( frameTimeMs > 0.0f ) {
                return frameTimeMs;
            }
        }

        return Time.unscaledDeltaTime * 1000.0f;
    }

}
