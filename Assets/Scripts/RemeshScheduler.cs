// decides when the interactive editor contours by itself, from a signature of what it would contour now and the time.
// It knows nothing of Unity, so tests can drive it with a clock of their own. Times are in seconds.
// - Changes are debounced: a contour waits until the signature has stayed the same for the debounce interval.
// - While changes keep coming in, as when a handle or slider is dragged, it contours again anyway if the last contour
//   took at most liveLimit, and no sooner than that contour's own time after it ended, so at most half the time goes
//   into contouring.
// - After a contour that took longer than pauseLimit it stops contouring by itself, until a contour is faster again.
// - Implementations that are not automatic, the managed ones, are never contoured by it.
// - Nothing is contoured by it while Unity is not the active application, where everything runs many times slower,
//   and contours timed then are not used for the decisions above
public sealed class RemeshScheduler {

    public enum State {
        UpToDate,   // the mesh shown is of what would be contoured now
        Due,        // contour now
        Waiting,    // changes are coming in, contour once they settle
        Manual,     // out of date, and only contoured when asked to
        Paused,     // out of date, and the last contour took too long to contour by itself
        Background  // out of date, and Unity is not the active application
    }

    public const double liveLimit  = 0.05;
    public const double pauseLimit = 1.0;

    public double debounce = 0.1;

    private string shownSignature;
    private string seenSignature;
    private double changedAt      = double.NegativeInfinity;
    private double contourEndedAt = double.NegativeInfinity;

    // how long the last contour took that was timed with Unity active, null until there is one
    public double? lastDuration { get; private set; }

    public bool paused {
        get { return this.lastDuration > pauseLimit; }
    }

    // the mesh shown is of the signature, as it is after a script reload
    public void shown( string signature ) {
        this.shownSignature = signature;
        this.seenSignature  = signature;
        this.changedAt      = double.NegativeInfinity;
    }

    // nothing is shown, as when a scene opens: the first update that may contour does, without waiting for changes to
    // settle
    public void nothingShown( string signature ) {
        this.shownSignature = null;
        this.seenSignature  = signature;
        this.changedAt      = double.NegativeInfinity;
    }

    // a contour of the signature ended at the given time, by itself or when asked to
    public void contoured( double now, string signature, double duration, bool active ) {
        this.shownSignature = signature;
        this.contourEndedAt = now;

        if( active ) {
            this.lastDuration = duration;
        }
    }

    // what to do at the given time, for what would be contoured now
    public State update( double now, string signature, bool automatic, bool active ) {
        if( signature != this.seenSignature ) {
            this.seenSignature = signature;
            this.changedAt     = now;
        }

        if( signature == this.shownSignature ) {
            return State.UpToDate;
        }
        if( !automatic ) {
            return State.Manual;
        }
        if( this.paused ) {
            return State.Paused;
        }
        if( !active ) {
            return State.Background;
        }

        var settled = now - this.changedAt >= this.debounce;
        var live    = this.lastDuration is double duration && duration <= liveLimit && now - this.contourEndedAt >= duration;

        return settled || live ? State.Due : State.Waiting;
    }

}
