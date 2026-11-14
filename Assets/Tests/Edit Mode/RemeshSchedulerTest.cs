using NUnit.Framework;

using State = RemeshScheduler.State;

// when the interactive editor contours by itself, on a clock of the test's own. Signatures are single letters, times
// are in seconds, and the debounce interval is the default 0.1 s
public class RemeshSchedulerTest {

    private const bool automatic = true;
    private const bool manual    = false;
    private const bool active    = true;
    private const bool inactive  = false;

    // a scheduler showing a contour of "A" that took the given time, ended at time 0
    private static RemeshScheduler showing( double duration ) {
        var scheduler = new RemeshScheduler( );

        scheduler.nothingShown( "A" );
        scheduler.contoured( 0.0, "A", duration, active );

        Assert.AreEqual( State.UpToDate, scheduler.update( 0.0, "A", automatic, active ) );

        return scheduler;
    }

    [Test]
    public void nothingShownContoursAtTheFirstUpdate( ) {
        var scheduler = new RemeshScheduler( );

        scheduler.nothingShown( "A" );

        Assert.AreEqual( State.Due, scheduler.update( 5.0, "A", automatic, active ) );
    }

    [Test]
    public void aShownMeshIsUpToDate( ) {
        var scheduler = new RemeshScheduler( );

        scheduler.shown( "A" );

        Assert.AreEqual( State.UpToDate, scheduler.update( 5.0, "A", automatic, active ) );
    }

    [Test]
    public void aChangeContoursOnceItHasSettled( ) {
        var scheduler = showing( 0.2 );

        Assert.AreEqual( State.Waiting, scheduler.update( 1.0,    "B", automatic, active ) );
        Assert.AreEqual( State.Waiting, scheduler.update( 1.0625, "B", automatic, active ) );
        Assert.AreEqual( State.Due,     scheduler.update( 1.125,  "B", automatic, active ) );
    }

    [Test]
    public void aBurstOfChangesGivesOneContour( ) {
        var scheduler = showing( 0.2 );

        Assert.AreEqual( State.Waiting, scheduler.update( 1.00, "B", automatic, active ) );
        Assert.AreEqual( State.Waiting, scheduler.update( 1.03, "C", automatic, active ) );
        Assert.AreEqual( State.Waiting, scheduler.update( 1.06, "D", automatic, active ) );
        Assert.AreEqual( State.Waiting, scheduler.update( 1.09, "E", automatic, active ) );
        Assert.AreEqual( State.Waiting, scheduler.update( 1.18, "E", automatic, active ) );
        Assert.AreEqual( State.Due,     scheduler.update( 1.20, "E", automatic, active ) );

        scheduler.contoured( 1.39, "E", 0.2, active );

        Assert.AreEqual( State.UpToDate, scheduler.update( 1.40, "E", automatic, active ) );
    }

    [Test]
    public void changingBackBeforeTheContourIsUpToDate( ) {
        var scheduler = showing( 0.2 );

        Assert.AreEqual( State.Waiting,  scheduler.update( 1.00, "B", automatic, active ) );
        Assert.AreEqual( State.UpToDate, scheduler.update( 1.05, "A", automatic, active ) );
        Assert.AreEqual( State.UpToDate, scheduler.update( 2.00, "A", automatic, active ) );
    }

    // as when a slider is dragged: a contour of 1/64 s goes again 1/64 s after it ended. The times are binary
    // fractions, so differences of them come out exact
    [Test]
    public void continuousChangesKeepContouringWhenFast( ) {
        const double duration = 1.0 / 64.0;

        var scheduler = showing( duration );

        Assert.AreEqual( State.Waiting, scheduler.update( duration / 2, "B", automatic, active ) );
        Assert.AreEqual( State.Due,     scheduler.update( duration,     "C", automatic, active ) );

        scheduler.contoured( duration * 2, "C", duration, active );

        Assert.AreEqual( State.Waiting, scheduler.update( duration * 2.5, "D", automatic, active ) );
        Assert.AreEqual( State.Due,     scheduler.update( duration * 3,   "E", automatic, active ) );
    }

    [Test]
    public void continuousChangesWaitForThePauseWhenSlow( ) {
        var scheduler = showing( 0.06 );

        for( var step = 1; step <= 20; ++step ) {
            Assert.AreEqual( State.Waiting, scheduler.update( step * 0.05, $"B{step}", automatic, active ), $"step {step}" );
        }

        Assert.AreEqual( State.Due, scheduler.update( 1.125, "B20", automatic, active ) );
    }

    [Test]
    public void theLimitForContouringDuringChangesIsInclusive( ) {
        var scheduler = showing( RemeshScheduler.liveLimit );

        Assert.AreEqual( State.Due, scheduler.update( RemeshScheduler.liveLimit, "B", automatic, active ) );
    }

    [Test]
    public void manualImplementationsNeverContourByThemselves( ) {
        var scheduler = showing( 0.001 );

        Assert.AreEqual( State.Manual, scheduler.update( 1.0,   "B", manual, active ) );
        Assert.AreEqual( State.Manual, scheduler.update( 100.0, "B", manual, active ) );

        scheduler.contoured( 101.0, "B", 5.0, active );

        Assert.AreEqual( State.UpToDate, scheduler.update( 102.0, "B", manual, active ) );
    }

    [Test]
    public void nothingByItselfWhileUnityIsInTheBackground( ) {
        var scheduler = showing( 0.01 );

        Assert.AreEqual( State.Background, scheduler.update( 1.0,  "B", automatic, inactive ) );
        Assert.AreEqual( State.Background, scheduler.update( 10.0, "B", automatic, inactive ) );
        Assert.AreEqual( State.Due,        scheduler.update( 11.0, "B", automatic, active ) );
    }

    [Test]
    public void aSlowContourPausesContouringByItself( ) {
        var scheduler = showing( 1.5 );

        Assert.IsTrue( scheduler.paused );
        Assert.AreEqual( State.Paused, scheduler.update( 1.0,  "B", automatic, active ) );
        Assert.AreEqual( State.Paused, scheduler.update( 10.0, "B", automatic, active ) );
    }

    [Test]
    public void aFasterContourResumesContouringByItself( ) {
        var scheduler = showing( 1.5 );

        // "Contour" pressed, with the debug view turned off
        scheduler.contoured( 2.0, "B", 0.5, active );

        Assert.IsFalse( scheduler.paused );
        Assert.AreEqual( State.Waiting, scheduler.update( 3.0, "C", automatic, active ) );
        Assert.AreEqual( State.Due,     scheduler.update( 3.125, "C", automatic, active ) );
    }

    [Test]
    public void aContourOfExactlyTheLimitDoesNotPause( ) {
        var scheduler = showing( RemeshScheduler.pauseLimit );

        Assert.IsFalse( scheduler.paused );
    }

    // in the background everything runs many times slower, which says nothing about how long a contour takes
    [Test]
    public void contoursTimedInTheBackgroundAreNotUsed( ) {
        var scheduler = showing( 0.01 );

        scheduler.contoured( 1.0, "B", 2.0, inactive );

        Assert.IsFalse( scheduler.paused );
        Assert.AreEqual( 0.01, scheduler.lastDuration );
        Assert.AreEqual( State.Due, scheduler.update( 1.02, "C", automatic, active ), "still fast enough for changes that keep coming" );
    }

    // what a mesh was made from can be lost while the mesh is still shown. It is then of nothing, and is contoured
    // again by the rules for any change: at once where that is allowed, since nothing has changed lately
    [Test]
    public void aMeshThatIsOfNothingAnyMoreIsContouredAgain( ) {
        var scheduler = showing( 0.2 );

        Assert.AreEqual( State.UpToDate, scheduler.update( 5.0, "A", automatic, active ) );

        scheduler.invalidate( );

        Assert.AreEqual( State.Due, scheduler.update( 5.0, "A", automatic, active ) );

        scheduler.invalidate( );

        Assert.AreEqual( State.Manual,     scheduler.update( 6.0, "A", manual,    active ) );
        Assert.AreEqual( State.Background, scheduler.update( 6.0, "A", automatic, inactive ) );

        scheduler.contoured( 7.0, "A", 0.2, active );

        Assert.AreEqual( State.UpToDate, scheduler.update( 8.0, "A", automatic, active ) );
    }

    [Test]
    public void theDebounceIntervalCanBeSet( ) {
        var scheduler = showing( 0.2 );

        scheduler.debounce = 0.5;

        Assert.AreEqual( State.Waiting, scheduler.update( 1.0, "B", automatic, active ) );
        Assert.AreEqual( State.Waiting, scheduler.update( 1.4, "B", automatic, active ) );
        Assert.AreEqual( State.Due,     scheduler.update( 1.5, "B", automatic, active ) );
    }

}
