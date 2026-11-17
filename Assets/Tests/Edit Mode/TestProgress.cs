using System;
using System.IO;
using NUnit.Framework.Interfaces;
using UnityEngine.TestRunner;

[assembly: TestRunCallback( typeof( TestProgress ) )]

// writes the test that runs, and since when, to Temp/test_progress.txt as each starts and finishes. The editor answers
// nothing while tests run and writes its log at the end, so this is how a script outside it can tell a test that runs
// on from a run of many tests: the file stops changing
public class TestProgress : ITestRunCallback {

    private const string path = "Temp/test_progress.txt";

    private static void write( string state, string name ) {
        try {
            File.WriteAllText( path, $"{DateTimeOffset.UtcNow.ToUnixTimeMilliseconds( ) / 1000.0:F3}\t{state}\t{name}\n" );
        }
        catch( IOException ) {
        }
        catch( UnauthorizedAccessException ) {
        }
    }

    public void RunStarted( ITest testsToRun ) {
        write( "run", testsToRun.FullName );
    }

    public void RunFinished( ITestResult testResults ) {
        write( "done", testResults.FullName );
    }

    public void TestStarted( ITest test ) {
        write( "started", test.FullName );
    }

    public void TestFinished( ITestResult result ) {
        write( "finished", result.FullName );
    }

}
